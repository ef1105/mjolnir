#ifndef POLY3D_H
#define POLY3D_H

extern int gUseOpenGL;

int poly3d_Begin( void );
void poly3d_End( void );

struct TileCache;
typedef struct TileCache TileCache;

/**
 * @brief emit RGBA tuples to describe tile appearence and transparency
 *
 * @param pRGBA points to an output buffer.  Calback is responsible for writing (red,green,blue,alpha)
 *        for each pixel in the tile.  0xff means "wholly transparent; 0x00 means "wholly opaque"
 */
typedef void GetTileDataRGBA( int tileNumber, int color, unsigned char *pRGBA, void *pUserParam );

/**
 * for tilemap or sprite tile caches, pass zero for textureWidth, textureHeight
 */
TileCache *poly3d_CreateTileCache(
   int tile_width, int tile_height, GetTileDataRGBA getTileDataRGBA, void *pUserParam,
   int textureWidth, int textureHeight,
	int filter );

void poly3d_DirtyTileNumber( TileCache *pCache, int tilenumber );

void poly3d_DirtyColor( TileCache *pCache, int color );

void poly3d_DirtyAll( TileCache *pCache );

void poly3d_Begin2d( TileCache *pCache );

void poly3d_DrawTile( int tilenumber, int color, int xpos, int ypos, int width, int height, int flipx, int flipy );

void poly3d_Begin3d( void );

typedef struct
{
   float x,y,z;
   int u,v;
   float blend; /* 0->texel only; 1->color only */
   float red;
   float green;
   float blue;
} Poly3dVertex;

void poly3d_DrawTriangle( TileCache *pCache, int color, Poly3dVertex v[3] );

void poly3d_DrawQuad( TileCache *pCache, int color, Poly3dVertex v[4], float zbias );

void poly3d_Draw3dSprite( TileCache *pCache, int tilenumber, int color, int xpos, int ypos, int width, int height, float translucency );

void poly3d_Clip( float cx, float cy, float vw, float vh );
void poly3d_NoClip( void );

void poly3d_SetBackgroundColor( int red, int green, int blue );

#endif
