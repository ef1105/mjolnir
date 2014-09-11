/**
 * @file win_poly3d.cpp
 */
#include <math.h>
#include <Windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include "poly3d.h"
#include "win_poly3d.h"
#include <stdio.h> /* for printf */

static int mLogicalWidth;
static int mLogicalHeight;

static int SCREEN_WIDTH;
static int SCREEN_HEIGHT;

#define EMULATE_VIEWPORTS
#define MANAGE_TEXTURE_COLORS

#define NEAR_PLANE 10.0f
#define FAR_PLANE ((float)0x7fffffff) /* bad for prop cycle */

static HDC mhDC;
static HGLRC mhRC;

static void
EnableOpenGL(HWND hWnd, HDC * hDC, HGLRC * hRC)
{
	PIXELFORMATDESCRIPTOR pfd;
	int format;
   mhDC = GetDC( hWnd );
	ZeroMemory( &pfd, sizeof( pfd ) );
	pfd.nSize = sizeof( pfd );
	pfd.nVersion = 1;
	pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
	pfd.iPixelType = PFD_TYPE_RGBA;
	pfd.cColorBits = 24;
	pfd.cDepthBits = 16;
	pfd.iLayerType = PFD_MAIN_PLANE;
	format = ChoosePixelFormat( *hDC, &pfd );
	SetPixelFormat( *hDC, format, &pfd );
	*hRC = wglCreateContext( *hDC );
	wglMakeCurrent( *hDC, *hRC );	
} /* EnableOpenGL */

static void
DisableOpenGL(HWND hWnd, HDC hDC, HGLRC hRC)
{
	wglMakeCurrent( NULL, NULL );
	wglDeleteContext( hRC );
	ReleaseDC( hWnd, hDC );
} /* DisableOpenGL */

int
poly3d_Begin( void )
{
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
   return 0;
} /* poly3d_Begin */

void
poly3d_End( void )
{
   glFlush();
	SwapBuffers( mhDC );
} /* poly3d_End */

void
poly3d_ResizeWindow( int width, int height )
{
	SCREEN_WIDTH = width;
	SCREEN_HEIGHT = height;
   glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT );
	printf( "SCREEN SIZE=%d,%d\n", SCREEN_WIDTH, SCREEN_HEIGHT );
}

void poly3d_SetBackgroundColor( int red, int green, int blue )
{
	glClearColor( 
		(float)red/255.0f, 
		(float)green/255.0f, 
		(float)blue/255.0f, 
		0.0f );
}

int
poly3d_Init( HWND hWnd, int width, int height )
{
	mLogicalWidth = width;
	mLogicalHeight = height;
	printf( "logical size=%d,%d\n", width, height );

   EnableOpenGL( hWnd, &mhDC, &mhRC );

	poly3d_ResizeWindow( width, height );

   glClearColor( 0.0f, 0.0f, 0.0f, 0.0f );
   glEnable( GL_DEPTH_TEST );
// glEnable(GL_CULL_FACE);
	glCullFace( GL_BACK );
   return 0;
} /* poly3d_Init */

void
poly3d_Term( HWND hWnd )
{
	DisableOpenGL( hWnd, mhDC, mhRC );
} /* poly3d_Term */

struct CachedTile
{
	int tileNumber;
   int color;
   unsigned char bWhollyTransparent;
   int x,y; /* location in texture */
   float u0,v0,u1,v1; /* location in texture */
   struct CachedTile *pNextInBucket;
};

#define COLOR_BUCKETS 16
#define TILENUMBER_BUCKETS 256

struct TileCache
{
   int textureID;
   int tileWidth, tileHeight;
   int textureWidth, textureHeight;
   int logicalTextureWidth, logicalTextureHeight;
   void *pUserParam;
   GetTileDataRGBA *getTileDataRGBA;
	struct CachedTile *bucket[COLOR_BUCKETS][TILENUMBER_BUCKETS];
	struct CachedTile *pCachedTile;
   int numCachedTiles;
   int maxCachedTiles;
   unsigned char *pScratchBuffer;
};

static TileCache *mpCurrentTileCache;

void
poly3d_Begin2d( TileCache *pTileCache )
{
   float znear, zfar;
   mpCurrentTileCache = pTileCache;
   glMatrixMode( GL_PROJECTION );
   glLoadIdentity();
   
   znear = 0.0f;
   zfar = 1.0f;
   glDisable( GL_DEPTH_TEST );
   if(pTileCache==NULL)
	{
		glEnable(GL_CULL_FACE);
	   glOrtho(
		   0.0f, mLogicalWidth, /* left..right */
			mLogicalHeight, 0.0f, /* top..bottom */
			0,0x7fff );

	   glScalef(1.0f, 1.0f, -1.0f);
		return;
	}
   glOrtho(
      0.0f, mLogicalWidth, /* left..right */
      mLogicalHeight, 0.0f, /* top..bottom */
      znear, zfar );

   glEnable(GL_TEXTURE_2D);
   glEnable(GL_ALPHA_TEST);
   glAlphaFunc(GL_GEQUAL, 0.05f);
   glBindTexture(GL_TEXTURE_2D,pTileCache->textureID);
}

static struct CachedTile *
FindCachedTile( struct TileCache *pCache, int tileNumber, int color )
{
   struct CachedTile *pCachedTile = pCache->bucket[color&(COLOR_BUCKETS-1)][tileNumber&(TILENUMBER_BUCKETS-1)];
   while( pCachedTile )
   {
      if( pCachedTile->tileNumber == tileNumber && pCachedTile->color == color )
      {
         break;
      }
      pCachedTile = pCachedTile->pNextInBucket;
   }
   return pCachedTile;
} /* FindCachedTile */

#define REDUCE 2

TileCache *
poly3d_CreateTileCache(
      int tileWidth, int tileHeight,
      GetTileDataRGBA getTileDataRGBA, void *pUserParam,
      int textureWidth, int textureHeight,
		int filter )
{
	float inset = filter?0.5f:0.0f;
	int texparam = filter?GL_LINEAR:GL_NEAREST;
   static int mTextureID;
   TileCache *pTileCache = malloc(sizeof(*pTileCache));
   if( pTileCache )
   {
      memset( pTileCache, 0, sizeof(*pTileCache) );
      pTileCache->pScratchBuffer = malloc(tileWidth*tileHeight*4);
      if( pTileCache->pScratchBuffer )
      {
         if( textureWidth||textureHeight )
         {
            pTileCache->logicalTextureWidth = textureWidth;
            pTileCache->logicalTextureHeight = textureHeight;
#ifdef REDUCE
            textureWidth>>=REDUCE;
            textureHeight>>=REDUCE;
            tileWidth>>=REDUCE;
            tileHeight>>=REDUCE;
#endif
         }
         if( !textureWidth )
         {
            textureWidth = 512;
         }
         if( !textureHeight )
         {
            textureHeight = 512;
         }
         pTileCache->textureWidth = textureWidth;
         pTileCache->textureHeight = textureHeight;
         pTileCache->tileWidth = tileWidth;
         pTileCache->tileHeight = tileHeight;
         pTileCache->getTileDataRGBA = getTileDataRGBA;
         pTileCache->pUserParam = pUserParam;
         pTileCache->maxCachedTiles = (textureWidth/tileWidth)*(textureHeight/tileHeight);
         pTileCache->pCachedTile = malloc(pTileCache->maxCachedTiles*sizeof(struct CachedTile));
         if( pTileCache->pCachedTile )
         {
            size_t numBytes = textureHeight*textureWidth*4;
            void *pTemp = malloc( numBytes );
            if( pTemp )
            {
               int x,y,i = 0;
               for( y=0; y<pTileCache->textureHeight; y+=tileHeight )
               {
                  for( x=0; x<pTileCache->textureWidth; x+=tileWidth )
                  {
                     struct CachedTile *pCachedTile = &pTileCache->pCachedTile[i++];
                     pCachedTile->x = x;
                     pCachedTile->y = y;
                     pCachedTile->u0 = (x+inset)/(float)pTileCache->textureWidth;
                     pCachedTile->v0 = (y+inset)/(float)pTileCache->textureHeight;
                     pCachedTile->u1 = (x+tileWidth-inset)/(float)pTileCache->textureWidth;
                     pCachedTile->v1 = (y+tileHeight-inset)/(float)pTileCache->textureHeight;
                     pCachedTile->color = -1;
                     pCachedTile->tileNumber = -1;
                  }
               }

               memset( pTemp, 0x00, numBytes );
               pTileCache->textureID = ++mTextureID;
               glBindTexture(GL_TEXTURE_2D, pTileCache->textureID);
               glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
               glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP/*GL_REPEAT*/);
               glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP/*GL_REPEAT*/);
               glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, texparam );
               glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, texparam );
               glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
               glTexImage2D(
                  GL_TEXTURE_2D, /* target */
                  0, /* level of detail */
                  GL_RGBA,
                  pTileCache->textureWidth,
                  pTileCache->textureHeight,
                  0, /* border */
                  GL_RGBA, /* format */
                  GL_UNSIGNED_BYTE, /* type */
                  pTemp );
               free( pTemp );
               return pTileCache;
            }
            free( pTileCache->pCachedTile );
         }
         free( pTileCache->pScratchBuffer );
      }
      free( pTileCache );
   }
   return NULL;
} /* poly3d_CreateTileCache */

static void
FlushCache( TileCache *pCache )
{
   int tileNumber;
   for( tileNumber=0; tileNumber<TILENUMBER_BUCKETS; tileNumber++ )
   {
      int color;
      for( color=0; color<COLOR_BUCKETS; color++ )
      {
         pCache->bucket[color][tileNumber] = NULL;
      }
   }
   pCache->numCachedTiles = 0;
} /* FlushCache */

static struct CachedTile *
GetCachedTile( TileCache *pCache, int tileNumber, int color )
{
   struct CachedTile *pCachedTile = FindCachedTile( pCache, tileNumber, color );
   if( !pCachedTile )
   {
      int i;
      int pixelsPerTile = pCache->tileWidth*pCache->tileHeight;
      if( pCache->numCachedTiles==pCache->maxCachedTiles )
      {
         FlushCache( pCache );
		}
      pCachedTile = &pCache->pCachedTile[pCache->numCachedTiles++];
		pCache->getTileDataRGBA( tileNumber, color, pCache->pScratchBuffer, pCache->pUserParam );
		pCachedTile->tileNumber = tileNumber;
      pCachedTile->color = color;
      pCachedTile->pNextInBucket = pCache->bucket[color&(COLOR_BUCKETS-1)][tileNumber&(TILENUMBER_BUCKETS-1)];
      pCache->bucket[color&(COLOR_BUCKETS-1)][tileNumber&(TILENUMBER_BUCKETS-1)] = pCachedTile;
      pCachedTile->bWhollyTransparent = 1;
		for( i=0; i<pixelsPerTile; i++ )
      {
         if( pCache->pScratchBuffer[i*4+3] != 0x00 )
         {
            pCachedTile->bWhollyTransparent = 0;
            glTexSubImage2D(
               GL_TEXTURE_2D,
               0, /* mipmap level */
               pCachedTile->x,pCachedTile->y,pCache->tileWidth,pCache->tileHeight,
               GL_RGBA,
               GL_UNSIGNED_BYTE,
               pCache->pScratchBuffer );
				break;
         }
      }
   }
   return pCachedTile;
} /* GetCachedTile */

/**
 * todo:
 * - fix scale glitches
 * - support xyflip
 * - add optional z coord for sprite-tilemap orthogonal
 */
void
poly3d_DrawTile( int tileNumber, int color, int xpos, int ypos, int width, int height, int flipx, int flipy )
{
   struct CachedTile *pCachedTile = GetCachedTile( mpCurrentTileCache, tileNumber, color );
   if( !pCachedTile->bWhollyTransparent )
   {
      glBegin( GL_QUADS );
      glColor3f( 1.0f, 1.0f, 1.0f );
      glTexCoord2f( flipx?pCachedTile->u1:pCachedTile->u0, flipy?pCachedTile->v0:pCachedTile->v1 );
      glVertex2d( xpos, ypos+height );
      glColor3f( 1.0f, 1.0f, 1.0f );
      glTexCoord2f( flipx?pCachedTile->u0:pCachedTile->u1, flipy?pCachedTile->v0:pCachedTile->v1 );
      glVertex2d( xpos+width, ypos+height );
      glColor3f( 1.0f, 1.0f, 1.0f );
      glTexCoord2f( flipx?pCachedTile->u0:pCachedTile->u1, flipy?pCachedTile->v1:pCachedTile->v0 );
      glVertex2d( xpos+width, ypos );
      glColor3f( 1.0f, 1.0f, 1.0f );
      glTexCoord2f( flipx?pCachedTile->u1:pCachedTile->u0, flipy?pCachedTile->v1:pCachedTile->v0 );
      glVertex2d( xpos, ypos );
      glEnd();
   }
} /* poly3d_DrawTile */

void
poly3d_Draw3dSprite( TileCache *pCache, int tileNumber, int color, int xpos, int ypos, int width, int height, float translucency )
{
   struct CachedTile *pCachedTile;

   glBindTexture(GL_TEXTURE_2D,pCache->textureID);
   pCachedTile = GetCachedTile( pCache, tileNumber, color );
   if( !pCachedTile->bWhollyTransparent )
   {
      float zoom = 1.0f;
      float z = NEAR_PLANE;
      float x = (xpos-320)*z/zoom;
      float y = (240-ypos)*z/zoom;
      float w = width*z/zoom;
      float h = -height*z/zoom;
   
      glEnable(GL_ALPHA_TEST);
      glAlphaFunc(GL_GEQUAL, 0.05f);

      glEnable (GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

      glBegin( GL_QUADS );

translucency = 1.0f-translucency;

      glColor4f( 1.0f, 1.0f, 1.0f, translucency );
      glTexCoord2f( pCachedTile->u0, pCachedTile->v0 );
      glVertex3f( x, y, z );

      glColor4f( 1.0f, 1.0f, 1.0f, translucency );
      glTexCoord2f( pCachedTile->u1, pCachedTile->v0 );
      glVertex3f( x+w, y, z );

      glColor4f( 1.0f, 1.0f, 1.0f, translucency );
      glTexCoord2f( pCachedTile->u1, pCachedTile->v1 );
      glVertex3f( x+w, y+h, z );

      glColor4f( 1.0f, 1.0f, 1.0f, translucency );
      glTexCoord2f( pCachedTile->u0, pCachedTile->v1 );
      glVertex3f( x, y+h, z );

      glEnd();

      glDisable (GL_BLEND);
   }
} /* poly3d_Draw3dSprite */

void
poly3d_DirtyTileNumber( TileCache *pCache, int tileNumber )
{
   int color;
   for( color=0; color<COLOR_BUCKETS; color++ )
   {
      struct CachedTile *prev = NULL;
      struct CachedTile *pCachedTile = pCache->bucket[color][tileNumber&(TILENUMBER_BUCKETS-1)];
      while( pCachedTile )
      {
         struct CachedTile *next = pCachedTile->pNextInBucket;
         if( pCachedTile->tileNumber == tileNumber )
         {
            if( prev )
            {
               prev->pNextInBucket = next;
            }
            else
            {
               pCache->bucket[color][tileNumber&(TILENUMBER_BUCKETS-1)] = next;
            }
         }
         else
         {
            prev = pCachedTile;
         }
         pCachedTile = next;
      }
   }
} /* poly3d_DirtyTileNumber */

void
poly3d_DirtyColor( TileCache *pCache, int color )
{
   int tileNumber;
   for( tileNumber=0; tileNumber<TILENUMBER_BUCKETS; tileNumber++ )
   {
      struct CachedTile *prev = NULL;
      struct CachedTile *pCachedTile = pCache->bucket[color&(COLOR_BUCKETS-1)][tileNumber];
      while( pCachedTile )
      {
         struct CachedTile *next = pCachedTile->pNextInBucket;
         if( pCachedTile->color == color )
         {
            if( prev )
            {
               prev->pNextInBucket = next;
            }
            else
            {
               pCache->bucket[color][tileNumber&(TILENUMBER_BUCKETS-1)] = next;
            }
         }
         else
         {
            prev = pCachedTile;
         }
         pCachedTile = next;
      }
   }
} /* poly3d_DirtyColor */

void
poly3d_DirtyAll( TileCache *pCache )
{
   int i,tileNumber;
   for( tileNumber=0; tileNumber<TILENUMBER_BUCKETS; tileNumber++ )
   {
      int color;
      for( color=0; color<COLOR_BUCKETS; color++ )
      {
         pCache->bucket[color][tileNumber] = NULL;
      }
   }
   for( i=0; i<pCache->maxCachedTiles; i++ )
   {
      struct CachedTile *pCachedTile = &pCache->pCachedTile[i];
      pCachedTile->color = -1;
      pCachedTile->tileNumber = -1;
   }
}

void
poly3d_Begin3d( void )
{
   float cx, cy;
	float zoom = 1.0f;

   glMatrixMode( GL_PROJECTION );

   glLoadIdentity();
   cx = mLogicalWidth/2;
	cy = mLogicalHeight/2;
	glFrustum(
      (0-cx)*NEAR_PLANE/zoom, /* left */
      (mLogicalWidth-cx)*NEAR_PLANE/zoom, /* right */
      (0-cy)*NEAR_PLANE/zoom, /* top */
      (mLogicalHeight-cy)*NEAR_PLANE/zoom, /* bottom */
      NEAR_PLANE,
      FAR_PLANE );
   glScalef(1.0f, 1.0f, -1.0f);

   glDisable( GL_DEPTH_TEST );
} /* poly3d_Begin3d */

static void
UpdateTexture( TileCache *pCache, int color, int umin, int umax, int vmin, int vmax )
{
   int u,v;
   int numCols = pCache->textureWidth/pCache->tileWidth;
   glEnable(GL_TEXTURE_2D);
   glBindTexture(GL_TEXTURE_2D,pCache->textureID);

   umin /= pCache->tileWidth;
   umax /= pCache->tileWidth;
   vmin /= pCache->tileHeight;
   vmax /= pCache->tileHeight;
#ifdef REDUCE
   umin>>=REDUCE;
   umax>>=REDUCE;
   vmin>>=REDUCE;
   vmax>>=REDUCE;
#endif

   for( v=vmin; v<=vmax; v++ )
   {
      for( u=umin; u<=umax; u++ )
      {
         int tileIndex = v*numCols+u;
         struct CachedTile *pCachedTile = &pCache->pCachedTile[tileIndex];
#ifdef MANAGE_TEXTURE_COLORS
         if( pCachedTile->color != color )
#else
         if( pCachedTile->color < 0 )
#endif
         {
            if( pCachedTile->color>=0 )
            {
               //printf( "(%d,%d)%x->%x\n", u,v,pCachedTile->color, color);
            }


            pCache->getTileDataRGBA( tileIndex, color, pCache->pScratchBuffer, pCache->pUserParam );
#ifdef REDUCE
            {
               unsigned char *pSrc = pCache->pScratchBuffer;
               unsigned char *pDst = pCache->pScratchBuffer;
               int x,y;
               int dx = ((1<<REDUCE)-1)*4;
               int dy = ((1<<REDUCE)-1)*4*(pCache->tileWidth<<REDUCE);
               for( y=0; y<pCache->tileHeight; y++ )
               {
                  for( x=0; x<pCache->tileWidth; x++ )
                  {
                     *pDst++ = *pSrc++;
                     *pDst++ = *pSrc++;
                     *pDst++ = *pSrc++;
                     *pDst++ = *pSrc++;
                     pSrc += dx;
                  }
                  pSrc += dy;
               }
            }
#endif

            glTexSubImage2D(
               GL_TEXTURE_2D,
               0, /* mipmap level */
               pCachedTile->x,pCachedTile->y,pCache->tileWidth, pCache->tileHeight,
               GL_RGBA,
               GL_UNSIGNED_BYTE,
               pCache->pScratchBuffer );

            pCachedTile->color = color;
         }
      }
   }
} /* UpdateTexture */

void
poly3d_DrawQuad( TileCache *pCache, int color, Poly3dVertex v[4], float zbias )
{
   unsigned int umin = ~0;
   unsigned int vmin = ~0;
   unsigned int umax = 0;
   unsigned int vmax = 0;
   float intensity[4];
   int bShadingCanBeModeledWithIntensityOnly = 1;
   int i;
	if( pCache )
	{
	   for( i=0; i<4; i++ )
	   {
	      Poly3dVertex *p = &v[i];
	      int tu = p->u;
	      int tv = p->v;
	      if( tu<umin ) umin = tu;
	      if( tv<vmin ) vmin = tv;
	      if( tu>umax ) umax = tu;
	      if( tv>vmax ) vmax = tv;
	      if( p->red == 0.0f && p->green == 0.0f && p->blue == 0.0f )
	      {
	         intensity[i] = 1.0f - p->blend;
	      }
	      else
	      {
	         bShadingCanBeModeledWithIntensityOnly = 0;
	      }
	   }
	   UpdateTexture( pCache, color, umin, umax, vmin, vmax );
	}
	else
	{
	   glDisable(GL_TEXTURE_2D);
		glEnable( GL_DEPTH_TEST );
	}
   glDisable(GL_ALPHA_TEST);

   glBegin( GL_QUADS );
 	for( i=0; i<4; i++ )
 	{
      Poly3dVertex *p = &v[i];
		if( pCache )
		{
	      float tu = (p->u+0.5f)/(float)pCache->logicalTextureWidth;
	      float tv = (p->v+0.5f)/(float)pCache->logicalTextureHeight;
	      if( bShadingCanBeModeledWithIntensityOnly )
	      {
	         glColor3f( intensity[i],intensity[i],intensity[i] );
	      }
	      else
	      {
	         glColor3f( 1.0f,1.0f,1.0f );
	      }
	      glTexCoord2f( tu,tv );
		}
		else
		{
			glColor3f( p->red, p->green, p->blue );
		}
      glVertex3f( v[i].x, v[i].y, v[i].z );
   }
   glEnd();

   if( !bShadingCanBeModeledWithIntensityOnly )
   {
      glDisable(GL_TEXTURE_2D);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
   
      glBegin( GL_QUADS );
 	   for( i=0; i<4; i++ )
 	   {
         glColor4f( v[i].red, v[i].green, v[i].blue, v[i].blend );
         glVertex3f( v[i].x, v[i].y, v[i].z );
      }
      glEnd();

      glDisable( GL_BLEND );
      glEnable(GL_TEXTURE_2D);
   }
} /* DrawPolyHelper */

#ifdef EMULATE_VIEWPORTS
static struct
{
   int bEnable;
   float x,y,w,h;
} mClip;
#endif

void
poly3d_Clip( float vx, float vy, float vw, float vh )
{
#ifdef EMULATE_VIEWPORTS
   if( mClip.bEnable!=1 || vx!=mClip.x || vy!=mClip.y || vw!=mClip.w || vh!=mClip.h )
   { /* todo: use glFrustrum instead of viewport/scissor? */
      float x = mLogicalWidth/2 + vx + vw;
      float y = mLogicalHeight/2 - vy + vh;
      float w = -vw*2;
      float h = -vh*2;
      
      int cx = x+w/2;
      int cy = y+h/2;

      glViewport(
			(cx-320)*SCREEN_WIDTH/mLogicalWidth,
			(cy-240)*SCREEN_HEIGHT/mLogicalHeight,
			SCREEN_WIDTH, SCREEN_HEIGHT );
      
		glScissor(
			x*SCREEN_WIDTH/mLogicalWidth,
			y*SCREEN_HEIGHT/mLogicalHeight,
			w*SCREEN_WIDTH/mLogicalWidth,
			h*SCREEN_HEIGHT/mLogicalHeight );
      
		glEnable( GL_SCISSOR_TEST );
      mClip.bEnable=1;
      mClip.x = vx;
      mClip.y = vy;
      mClip.w = vw;
      mClip.h = vh;
   }
#endif
} /* poly3d_Clip */

void
poly3d_NoClip( void )
{
#ifdef EMULATE_VIEWPORTS
   if( mClip.bEnable!=0 )
   {
      glDisable(GL_SCISSOR_TEST);
      glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT );
      mClip.bEnable = 0;
   }
#endif
} /* poly3d_NoClip */
