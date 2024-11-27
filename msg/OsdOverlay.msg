# ASCII character and a pixel coordinate where the character should be drawn on screen.
# Used by the atxxxx OSD driver to define the overlay over the analog video image. 
# Used in topic /fmu/in/osd_overlay 

uint64 timestamp  # time since system start (microseconds)

uint32 x
uint32 y
uint8 c     # ASCII character to be overlayed on screen at (x,y)