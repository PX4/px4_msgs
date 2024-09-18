/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include "util.h"
#include <string>
#include <utility>
#include <vector>
#include <functional>
#include <memory>
#include <bitset>
#include <queue>
#include <optional>

// This implements a directed graph with potential cycles used for translation.
// There are 2 types of nodes: messages (e.g. publication/subscription endpoints) and
// translations. Translation nodes are always in between message nodes, and can have N input messages
// and M output messages.

struct MessageIdentifier {
	std::string topic_name;
	MessageVersionType version;

	bool operator==(const MessageIdentifier& other) const {
		return topic_name == other.topic_name && version == other.version;
	}
	bool operator!=(const MessageIdentifier& other) const {
		return !(*this == other);
	}
};

template<>
struct std::hash<MessageIdentifier>
{
	std::size_t operator()(const MessageIdentifier& s) const noexcept
	{
		std::size_t h1 = std::hash<std::string>{}(s.topic_name);
		std::size_t h2 = std::hash<std::uint32_t>{}(s.version);
		return h1 ^ (h2 << 1);
	}
};


using MessageBuffer = std::shared_ptr<void>;

template <typename NodeData, typename IdType>
class MessageNode;
template <typename NodeData, typename IdType>
class Graph;

template <typename NodeData, typename IdType>
using MessageNodePtrT = std::shared_ptr<MessageNode<NodeData, MessageIdentifier>>;

template <typename NodeData, typename IdType=MessageIdentifier>
class TranslationNode {
public:
	using TranslationCB = std::function<void(const std::vector<MessageBuffer>&, std::vector<MessageBuffer>&)>;

	TranslationNode(std::vector<MessageNodePtrT<NodeData, IdType>> inputs,
					std::vector<MessageNodePtrT<NodeData, IdType>> outputs,
					TranslationCB translation_db)
					: _inputs(std::move(inputs)), _outputs(std::move(outputs)), _translation_cb(std::move(translation_db)) {
		assert(_inputs.size() <= kMaxNumInputs);

		_input_buffers.resize(_inputs.size());
		for (unsigned i = 0; i < _inputs.size(); ++i) {
			_input_buffers[i] = _inputs[i]->buffer();
		}

		_output_buffers.resize(_outputs.size());
		for (unsigned i = 0; i < _outputs.size(); ++i) {
			_output_buffers[i] = _outputs[i]->buffer();
		}
	}

	void setInputReady(unsigned index) {
		_inputs_ready.set(index);
	}

	bool translate() {
		if (_inputs_ready.count() == _input_buffers.size()) {
			_translation_cb(_input_buffers, _output_buffers);
			_inputs_ready.reset();
			return true;
		}
		return false;
	}

	const std::vector<MessageNodePtrT<NodeData, IdType>>& inputs() const { return _inputs; }
	const std::vector<MessageNodePtrT<NodeData, IdType>>& outputs() const { return _outputs; }

private:
	static constexpr int kMaxNumInputs = 32;

	const std::vector<MessageNodePtrT<NodeData, IdType>> _inputs;
	std::vector<MessageBuffer> _input_buffers; ///< Cached buffers from _inputs.buffer()
	const std::vector<MessageNodePtrT<NodeData, IdType>> _outputs;
	std::vector<MessageBuffer> _output_buffers;
	const TranslationCB _translation_cb;

	std::bitset<kMaxNumInputs> _inputs_ready;
};

template <typename NodeData, typename IdType>
using TranslationNodePtrT = std::shared_ptr<TranslationNode<NodeData, MessageIdentifier>>;


template <typename NodeData, typename IdType=MessageIdentifier>
class MessageNode {
public:

	explicit MessageNode(NodeData node_data, size_t index, MessageBuffer  message_buffer)
	: _buffer(std::move(message_buffer)), _data(std::move(node_data)), _index(index) {}

	MessageBuffer& buffer() { return _buffer; }

	void addTranslationInput(TranslationNodePtrT<NodeData, IdType> node, unsigned input_index) {
		_translations.push_back(Translation{std::move(node), input_index});
	}

	NodeData& data() { return _data; }

	void resetNodes() {
		_translations.clear();
	}

private:
	struct Translation {
		TranslationNodePtrT<NodeData, IdType> node; ///< Counterpart to the TranslationNode::_inputs
		unsigned input_index; ///< Index into the TranslationNode::_inputs
	};
	MessageBuffer _buffer;
	std::vector<Translation> _translations;

	NodeData _data;

	const size_t _index;
	MessageNode<NodeData, IdType>* _iterating_previous{nullptr};
	bool _want_translation{false};

	friend class Graph<NodeData, IdType>;
};

template <typename NodeData, typename IdType=MessageIdentifier>
class Graph {
public:
	using MessageNodePtr = MessageNodePtrT<NodeData, IdType>;

	~Graph() {
		// Explicitly reset the nodes array to break up potential cycles and prevent memory leaks
		for (auto& [id, node] : _nodes) {
			node->resetNodes();
		}
	}

	/**
	 * @brief Add a message node if it does not exist already
	 */
	bool addNodeIfNotExists(const IdType& id, NodeData node_data, const MessageBuffer& message_buffer) {
		if (_nodes.find(id) != _nodes.end()) {
			return false;
		}
		// Node that we cannot remove nodes due to using the index as an array index
		const size_t index = _nodes.size();
		_nodes.insert({id, std::make_shared<MessageNode<NodeData, IdType>>(std::move(node_data), index, message_buffer)});
		return true;
	}

	/**
	 * @brief Add a translation edge with N inputs and M output nodes. All nodes must already exist.
	 */
	void addTranslation(const typename TranslationNode<NodeData, IdType>::TranslationCB& translation_cb,
						const std::vector<IdType>& inputs, const std::vector<IdType>& outputs) {
		auto init = [this](const std::vector<IdType>& from, std::vector<MessageNodePtrT<NodeData, IdType>>& to) {
			for (unsigned i=0; i < from.size(); ++i) {
				auto node_iter = _nodes.find(from[i]);
				assert(node_iter != _nodes.end());
				to[i] = node_iter->second;
			}
		};
		std::vector<MessageNodePtrT<NodeData, IdType>> input_nodes(inputs.size());
		init(inputs, input_nodes);
		std::vector<MessageNodePtrT<NodeData, IdType>> output_nodes(outputs.size());
		init(outputs, output_nodes);

		auto translation_node = std::make_shared<TranslationNode<NodeData, IdType>>(std::move(input_nodes), std::move(output_nodes), translation_cb);
		for (unsigned i=0; i < translation_node->inputs().size(); ++i) {
			translation_node->inputs()[i]->addTranslationInput(translation_node, i);
		}
	}


	/**
	 * @brief Translate a message node in the graph.
	 *
	 * This function performs a two-pass translation of a message node in the graph.
	 * First, it finds the required nodes that need the translation results, and then
	 * it runs the translation on these nodes to prevent unnecessary message conversions.
	 *
	 * @param node The message node to translate.
	 * @param node_requires_translation_result A callback function that determines whether a node requires the translation result.
	 * @param on_translated A callback function that is called for translated nodes (with an updated message buffer).
	 */
	void translate(const MessageNodePtr& node, const std::function<bool(const MessageNodePtr&)>& node_requires_translation_result,
				   const std::function<void(const MessageNodePtr&)>& on_translated) {
		// Do translation in 2 passes: first, find the required nodes that require the translation results,
		// then run the translation on these nodes to prevent unnecessary message conversions
		// (the assumption here is that conversions are more expensive than iterating the graph)
		prepareTranslation(node, node_requires_translation_result);
		runTranslation(node, on_translated);
	}

	std::optional<MessageNodePtr> findNode(const IdType& id) const {
		auto iter = _nodes.find(id);
		if (iter == _nodes.end()) {
			return std::nullopt;
		}
		return iter->second;
	}

	void iterateNodes(const std::function<void(const IdType& type, const MessageNodePtr& node)>& cb) const {
		for (const auto& [id, node] : _nodes) {
			cb(id, node);
		}
	}

	/**
	 * Iterate all reachable nodes from a given node using the BFS (shortest path) algorithm
	 */
	void iterateBFS(const MessageNodePtr& node, const std::function<void(const MessageNodePtr&)>& cb) {
		_node_visited.resize(_nodes.size());
		std::fill(_node_visited.begin(), _node_visited.end(), false);

		std::queue<MessageNodePtr> queue;
		_node_visited[node->_index] = true;
		node->_iterating_previous = nullptr;
		queue.push(node);
		cb(node);

		while (!queue.empty()) {
			MessageNodePtr current = queue.front();
			queue.pop();
			for (auto& translation : current->_translations) {
				for (auto& next_node : translation.node->outputs()) {
					if (_node_visited[next_node->_index]) {
						continue;
					}
					_node_visited[next_node->_index] = true;
					next_node->_iterating_previous = current.get();
					queue.push(next_node);

					cb(next_node);
				}
			}
		}
	}


private:
	void prepareTranslation(const MessageNodePtr& node, const std::function<bool(const MessageNodePtr&)>& node_requires_translation_result) {
		iterateBFS(node, [&](const MessageNodePtr& node) {
			if (node_requires_translation_result(node)) {
				auto* previous_node = node.get();
				while (previous_node) {
					previous_node->_want_translation = true;
					previous_node = previous_node->_iterating_previous;
				}
			}
		});
	}

	void runTranslation(const MessageNodePtr& node, const std::function<void(const MessageNodePtr&)>& on_translated) {
		_node_had_update.resize(_nodes.size());
		std::fill(_node_had_update.begin(), _node_had_update.end(), false);
		_node_had_update[node->_index] = true;

		iterateBFS(node, [&](const MessageNodePtr& node) {
			// If there was no update for this node, there's nothing to do (i.e. want_translation is false or the
			// message buffer did not change)
			if (!_node_had_update[node->_index]) {
				return;
			}

			on_translated(node);

			if (node->_want_translation) {
				node->_want_translation = false;
				for (auto &translation : node->_translations) {
					// Skip translation if none of the output nodes has _want_translation set or
					// if any of the nodes already had an update.
					// This also prevents translating 'backwards' by one step (from where we came from)
					bool want_translation = false;
					bool had_update = false;
					for (auto &next_node: translation.node->outputs()) {
						want_translation |= next_node->_want_translation;
						had_update |= _node_had_update[next_node->_index];
					}
					if (!want_translation || had_update) {
						continue;
					}
					translation.node->setInputReady(translation.input_index);
					if (translation.node->translate()) {
						for (auto &next_node: translation.node->outputs()) {
							_node_had_update[next_node->_index] = true;
						}
					}
				}
			}
		});
	}

	std::unordered_map<IdType, MessageNodePtr> _nodes;
	std::vector<bool> _node_visited; ///< Cached, to avoid the need to re-allocate on each iteration
	std::vector<bool> _node_had_update; ///< Cached, to avoid the need to re-allocate on each iteration
};