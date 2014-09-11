Phil Stroffolino
http://www.machna.com/games

Mjolnir is a MAME (http://www.mamedev.org) derivative work, based on a branch of MAME0100.

Mjolnir supports a subset of games released for Namco's System 21, System 22, and Super System 22 harware.

The focus to date has been on Prop Cycle and Starblade, though other titles work to various extents and bug reports are welcome.

Mjolnir's primry purpose is to provide a framework for experimenting with rendering abstractions that provide a bridge between traditional arcade video hardware and modern APIs like OpenGL and Direct3d.

Mjolnir's secondary purpose is to explore the benefits of targeted patches and recompilation techniques to eliminate performance bottlenecks.


To build, unzip over mame0.100. Please make sure you can build a baseline mame0.100 build before reporting any build problems.


Mjolnir 0.4 - October 31, 2007
- Solvalou working
- improved gradient layer simulation (redeye planet surface, solvalou horizon)
- suport for scaled primitives
- backface culling; fixes several priority effects

Mjolnir 0.3.1 - October 13, 2007
- System22 (Prop Cycle) texture filtering had been broken in the 0.3 release; this caused textures to look "chunky" and has been fixed in this interim release
- fixes for fullscreen support & window resizing

Mjolnir 0.3 - October 12, 2007
- fixed in-game Starblade crashes
- added fullscreen support
- prelim gradient support

Mjolnir 0.2 - October 3, 2007
- added protection for out-of-bounds memory read from pointram (System21)

Mjolnir 0.1 - October 1, 2007

Known Issues:
- MAME's "UI" layer is functional, but invisible
- System21: incomplete emulation of the gradient-style backdrop
- System21: incomplete near-plane clipping
- System21: Air Combat & Cybersled: graphics glitches
- System21: Solvalou water - wrong color
- System21: Winning Run: missing bitmap layer
- System22: Prop Cycle locks up after entering a high score
- System22 titles run poorly with autoframeskip