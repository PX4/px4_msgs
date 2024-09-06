#include "translations.h"

void TranslationForTopic::registerVersion(DirectTranslationData data) {
	_direct_translations.emplace_back(std::move(data));
}

void Translations::registerDirectTranslation(const std::string &topic_name, DirectTranslationData data) {
	auto iter = _topic_translations.find(topic_name);
	if (iter == _topic_translations.end()) {
		auto [iter_inserted, _] = _topic_translations.emplace(topic_name, TranslationForTopic(topic_name));
		iter = iter_inserted;
	}
	iter->second.registerVersion(std::move(data));
}
