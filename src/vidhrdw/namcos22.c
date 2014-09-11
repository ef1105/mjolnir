/**
 * video hardware for Namco System22
 *
 * todo:
 *
 * - sync with 102s
 *
 * - air combat22: bad fader, game crashes
// cpu #0 (PC=000201A6): unmapped program memory dword write to 00C20000 = 00007FFF & FFFFFFFF
// cpu #1 (PC=0000920F): warning - op-code execute on mapped I/O

 * - victory lap: no polygons
 * - alpine surfer: not working at all
 * - compare slave dsp code
 *
 * - sprite clipping to window
 * - time crisis title screen: bogus sprite tiles
 * - alpine racer: sprite xy offset
 * - text, sprite fade
 * - fogged sprite support
 * - sprite priority over text
 * - spot ram: bitmap for headlights
 * - sys22 fog, fader
 */
#include "namcos22.h"
#include <math.h>
#include "poly3d.h"
#include "cpu/tms32025/tms32025.h"

#define DSP_FIXED_TO_FLOAT( X ) (((INT16)(X))/(float)0x7fff)
#define SPRITERAM_SIZE (0x9b0000-0x980000)
#define CGRAM_SIZE 0x1e000
#define NUM_CG_CHARS ((CGRAM_SIZE*8)/(64*16)) /* 0x3c0 */

/* 16 bit access to DSP RAM */
static UINT16 namcos22_dspram_bank;
static UINT16 mUpperWordLatch;

static int mbDSPisActive;
static int mbSuperSystem22; /* used to conditionally support Super System22-specific features */

/* modal rendering properties */
static INT32 mAbsolutePriority;
static INT32 mObjectShiftValue22;

static UINT16 mPrimitiveID; /* 3d primitive to render */

static float mViewMatrix[4][4];

static void
matrix3d_Multiply( float A[4][4], float B[4][4] )
{
	float temp[4][4];
	int row,col;

	for( row=0;row<4;row++ )
	{
		for(col=0;col<4;col++)
		{
			float sum = 0.0;
			int i;
			for( i=0; i<4; i++ )
			{
				sum += A[row][i]*B[i][col];
			}
			temp[row][col] = sum;
		}
	}
	memcpy( A, temp, sizeof(temp) );
} /* matrix3d_Multiply */

static void
matrix3d_Identity( float M[4][4] )
{
	int r,c;

	for( r=0; r<4; r++ )
	{
		for( c=0; c<4; c++ )
		{
			M[r][c] = (r==c)?1.0:0.0;
		}
	}
} /* matrix3d_Identity */

static void
TransformPoint( float *vx, float *vy, float *vz, float m[4][4] )
{
	float x = *vx;
	float y = *vy;
	float z = *vz;
	*vx = m[0][0]*x + m[1][0]*y + m[2][0]*z + m[3][0];
	*vy = m[0][1]*x + m[1][1]*y + m[2][1]*z + m[3][1];
	*vz = m[0][2]*x + m[1][2]*y + m[2][2]*z + m[3][2];
}

static void
TransformNormal( float *nx, float *ny, float *nz, float m[4][4] )
{
	float x = *nx;
	float y = *ny;
	float z = *nz;
	*nx = m[0][0]*x + m[1][0]*y + m[2][0]*z;
	*ny = m[0][1]*x + m[1][1]*y + m[2][1]*z;
	*nz = m[0][2]*x + m[1][2]*y + m[2][2]*z;
}

#define MAX_LIT_SURFACES 32
static UINT8 mLitSurfaceInfo[MAX_LIT_SURFACES];
static INT32 mSurfaceNormalFormat;

static unsigned mLitSurfaceCount;
static unsigned mLitSurfaceIndex;

static int mPtRomSize;
static const UINT8 *mpPolyH;
static const UINT8 *mpPolyM;
static const UINT8 *mpPolyL;

/* opengl resources */
static TileCache *mpSpriteTileCache;
static TileCache *mpTextTileCache;
static TileCache *mpTextureCache[16];

static struct
{
	float zoom, vx, vy, vw, vh;
	float lx,ly,lz; /* unit vector for light direction */
	int ambient; /* 0.0..1.0 */
	int power;	/* 0.0..1.0 */
} mCamera;

typedef enum
{
   eSCENENODE_NONLEAF,
   eSCENENODE_QUAD3D,
   eSCENENODE_SPRITE
} SceneNodeType;

#define RADIX_BITS 4
#define RADIX_BUCKETS (1<<RADIX_BITS)
#define RADIX_MASK (RADIX_BUCKETS-1)

struct SceneNode
{
   SceneNodeType type;   
   struct SceneNode *nextInBucket;
   union
   {
      struct
      {
         struct SceneNode *next[RADIX_BUCKETS];
      } nonleaf;

      struct
      {
         float vx,vy,vw,vh;
         struct TileCache *pTexture;
         int color;
         int flags;
         Poly3dVertex v[4];
      } quad3d;

      struct
      {
         int tile, color;
         int flipx, flipy;
	      int numcols, numrows;
	      int xpos, ypos;
	      int sizex, sizey;
         int translucency;
      } sprite;
   } data;
};
static struct SceneNode mSceneRoot;
static struct SceneNode *mpFreeSceneNode;

static UINT8
nthbyte( const UINT32 *pSource, int offs )
{
	pSource += offs/4;
	return (pSource[0]<<((offs&3)*8))>>24;
}

static UINT16
nthword( const UINT32 *pSource, int offs )
{
	pSource += offs/2;
	return (pSource[0]<<((offs&1)*16))>>16;
}

static struct SceneNode *
MallocSceneNode( void )
{
   static int maxNode;
   struct SceneNode *node = mpFreeSceneNode;
   if( node )
   { /* use free pool */
      mpFreeSceneNode = node->nextInBucket;
   }
   else
   {
      node = malloc(sizeof(struct SceneNode));
      if( node )
      {
         maxNode++;
      }
      else
      { /* insufficient mem */
         exit(1);
      }
   }
   memset( node, 0, sizeof(*node) );
   return node;
} /* MallocSceneNode */

static void
FreeSceneNode( struct SceneNode *node )
{
   node->nextInBucket = mpFreeSceneNode;
   mpFreeSceneNode = node;
} /* FreeSceneNode */

static struct SceneNode *
NewSceneNode( UINT32 zsortvalue24, SceneNodeType type )
{
   struct SceneNode *node = &mSceneRoot;
   int i;
   for( i=0; i<24; i+=RADIX_BITS )
   {
      int hash = (zsortvalue24>>20)&RADIX_MASK;
      struct SceneNode *next = node->data.nonleaf.next[hash];
      if( !next )
      {
         next = MallocSceneNode();
         next->type = eSCENENODE_NONLEAF;
         node->data.nonleaf.next[hash] = next;
      }
      node = next;
      zsortvalue24 <<= RADIX_BITS;
   }

   if( node->type == eSCENENODE_NONLEAF )
   {
      node->type = type;
      return node;
   }
   else
   {
      struct SceneNode *leaf = MallocSceneNode();
      leaf->type = type;
      leaf->nextInBucket = node->nextInBucket;
      node->nextInBucket = leaf;
      return leaf;
   }
} /* NewSceneNode */

static void
RenderSprite( struct SceneNode *node )
{
   int tile = node->data.sprite.tile;
   int col,row;
   for( row=0; row<node->data.sprite.numrows; row++ )
   {
      for( col=0; col<node->data.sprite.numcols; col++ )
   	{
         poly3d_Draw3dSprite(
               mpSpriteTileCache,
               tile,
               node->data.sprite.color,
               node->data.sprite.xpos+col*node->data.sprite.sizex, 
               node->data.sprite.ypos+row*node->data.sprite.sizey, 
               node->data.sprite.sizex,
               node->data.sprite.sizey,
               node->data.sprite.translucency/256.0f );
      	tile++;
      } /* next col */
	} /* next row */
} /* RenderSprite */

static void
RenderSceneHelper( struct SceneNode *node, int pri )
{
   if( node )
   {
      if( node->type == eSCENENODE_NONLEAF )
      {
         int i;
         for( i=RADIX_BUCKETS-1; i>=0; i-- )
         {
            RenderSceneHelper( node->data.nonleaf.next[i], pri ); 
         }
         if( pri )
         {
            FreeSceneNode( node );
         }
      }
      else
      {
         while( node )
         {
            struct SceneNode *next = node->nextInBucket;
            
            switch( node->type )
            {
            case eSCENENODE_QUAD3D:
               if( (node->data.quad3d.flags&0x2000) == pri*0x2000 )
               {
                  poly3d_Clip(
                     node->data.quad3d.vx,
                     node->data.quad3d.vy,
                     node->data.quad3d.vw,
                     node->data.quad3d.vh );
                  poly3d_DrawQuad(
                     node->data.quad3d.pTexture,
                     node->data.quad3d.color,
                     node->data.quad3d.v,
							0.0f );
               }
               break;

            case eSCENENODE_SPRITE:
               if( pri==0 )
               {
                  poly3d_NoClip();
                  RenderSprite( node );
               }
               break;

            default:
               break;
            }
            if( pri )
            {
               FreeSceneNode( node );
            }
            node = next;
         }
      }
   }
} /* RenderSceneHelper */

static void
RenderScene( int pri )
{
   struct SceneNode *node = &mSceneRoot;
   int i;
   poly3d_Begin3d();
   for( i=RADIX_BUCKETS-1; i>=0; i-- )
   {
      RenderSceneHelper( node->data.nonleaf.next[i], pri ); 
      if( pri )
      {
         node->data.nonleaf.next[i] = NULL;
      }
   }
   poly3d_NoClip();
} /* RenderScene */

static float
DspFloatToNativeFloat( UINT32 iVal )
{
   INT16 mantissa = (INT16)iVal;
	float result = (float)mantissa;
	int exponent = (iVal>>16)&0xff;
	while( exponent<0x2e )
	{
		result /= 2.0;
		exponent++;
	}
	return result;
} /* DspFloatToNativeFloat */

static INT32
GetPolyData( INT32 addr )
{
	INT32 result;
	if( addr<0 || addr>=mPtRomSize )
	{
		return -1; /* HACK */
	}
	result = (mpPolyH[addr]<<16)|(mpPolyM[addr]<<8)|mpPolyL[addr];
	if( result&0x00800000 )
	{
		result |= 0xff000000; /* sign extend */
	}
	return result;
} /* GetPolyData */

UINT32
namcos22_point_rom_r( offs_t offs )
{
	return GetPolyData(offs);
}

/* text layer uses a set of 16x16x8bpp tiles defined in RAM */
static gfx_layout cg_layout =
{
	16,16,
	NUM_CG_CHARS,
	4,
	{ 0,1,2,3 },
#ifdef LSB_FIRST
	{ 4*6,4*7, 4*4,4*5, 4*2,4*3, 4*0,4*1,
	  4*14,4*15, 4*12,4*13, 4*10,4*11, 4*8,4*9 },
#else
	{ 4*0,4*1,4*2,4*3,4*4,4*5,4*6,4*7,
	  4*8,4*9,4*10,4*11,4*12,4*13,4*14,4*15 },
#endif
	{ 64*0,64*1,64*2,64*3,64*4,64*5,64*6,64*7,64*8,64*9,64*10,64*11,64*12,64*13,64*14,64*15 },
	64*16
}; /* cg_layout */

UINT32 *namcos22_cgram;
UINT32 *namcos22_textram;
UINT32 *namcos22_polygonram;
UINT32 *namcos22_gamma;
UINT32 *namcos22_vics_data;
UINT32 *namcos22_vics_control;
UINT32 *namcos22_czattr;
UINT32 *namcos22_czram;
UINT32 *namcos22_tilemapattr;

static int cgsomethingisdirty;
static unsigned char *cgdirty;
static unsigned char *dirtypal;

static UINT16 *mpTextureTileMap16;
static UINT8 *mpTextureTileMapAttr;
static UINT8 *mpTextureTileData;
static UINT8 mXYAttrToPixel[16][16][16];

static void
InitXYAttrToPixel( void )
{
	unsigned attr,x,y,ix,iy,temp;
	for( attr=0; attr<16; attr++ )
	{
		for( y=0; y<16; y++ )
		{
			for( x=0; x<16; x++ )
			{
				ix = x; iy = y;
				if( attr&4 ) ix = 15-ix;
				if( attr&2 ) iy = 15-iy;
				if( attr&8 ){ temp = ix; ix = iy; iy = temp; }
				mXYAttrToPixel[attr][x][y] = (iy<<4)|ix;
			}
		}
	}
} /* InitXYAttrToPixel */

static void
PatchTexture( void )
{
	int i;
	switch( namcos22_gametype )
	{
	case NAMCOS22_RIDGE_RACER:
	case NAMCOS22_RIDGE_RACER2:
	case NAMCOS22_ACE_DRIVER:
	case NAMCOS22_CYBER_COMMANDO:
   	for( i=0; i<0x100000; i++ )
   	{
   		int tile = mpTextureTileMap16[i];
   		int attr = mpTextureTileMapAttr[i];
   		if( (attr&0x1)==0 )
   		{
   			tile = (tile&0x3fff)|0x8000;
   			mpTextureTileMap16[i] = tile;
   		}
   	}
		break;

	default:
   	break;
   }
} /* PatchTexture */

static void
GetTextureTileDataRGBA(int tileNumber, int color, unsigned char *pRGBA, void *pUserParam)
{
   const pen_t *pPal = &Machine->pens[color<<8];
   unsigned offs = tileNumber + 256*256*(int)pUserParam;
   unsigned tile = mpTextureTileMap16[offs];
   int x,y;
   for( y=0; y<16; y++ )
   {
      for( x=0; x<16; x++ )
      {
         int pen = mpTextureTileData[(tile<<8)|mXYAttrToPixel[mpTextureTileMapAttr[offs]][x][y]];
         UINT32 rgb = pPal[pen];
         *pRGBA++ = rgb>>16; /* red */
         *pRGBA++ = (rgb>>8)&0xff; /* green */
         *pRGBA++ = rgb&0xff; /* blue */
         *pRGBA++ = 0x00; /* opaque */
      }
   }
} /* GetTextureTileDataRGBA */

static TileCache *
GetTextureCache( int bank )
{
   if( !mpTextureCache[bank] )
   {
      mpTextureCache[bank] = poly3d_CreateTileCache( 16,16, GetTextureTileDataRGBA, (void *)bank, 0x1000, 0x1000, 1 );
   }
   return mpTextureCache[bank];
} /* GetTextureCache */

static UINT8
Clamp256( int v )
{
   if( v<0 )
   {
      v = 0;
   }
   else if( v>255 )
   {
      v = 255;
   }
   return v;
} /* Clamp256 */

static void
ApplyFade( Poly3dVertex *p, int intensity )
{
   int zc = Clamp256(p->z/0x2000);
	float red;
	float green;
	float blue;
   float blend;
   
   if( intensity>128 )
   { /* blend to white */
      red   = 1.0f;
      green = 1.0f;
      blue  = 1.0f;
      blend = (intensity-128)/(255.0f-128.0f);
   }
   else
   {
      red   = 0.0f;
      green = 0.0f;
      blue  = 0.0f;
      blend = (128-intensity)/128.0f;
   }

   if( namcos22_gamma && namcos22_gametype!=NAMCOS22_AIR_COMBAT22 )
   {
      /*                                                   |
00824000: ffffff00 00000000 1830407f 00800000 0000007f 0f00|00 00 00 00 03 7f 00010007 // trans sprite
00824000: ffffff00 00000000 3040307f 00000000 0080007f 0f00|00 00 00 00 03 7f 00010007 // trans poly
00824000: ffffff00 00000000 1800187f 00000000 0000007f 0f80|00 00 00 00 03 7f 00010007 // trans text
                     ^^^^^^                                |                           // backcolor rgb
                                                           |            ^^             // fader target
                            ^^^^^^     ^^       ^^       ^^|                           // 0x0d,0x11,0x15 ==?
                                                           |^^ ^^ ^^ ^^                // fade rgb,pct
*/
      int fadeTargetFlags = nthbyte( namcos22_gamma, 0x1a );
      if( namcos22_czram )
      {
         INT16 fog = nthword(namcos22_czattr, 0 );
         if( fog > 0 )
         {
            UINT16 fogDensity = nthword(namcos22_czram,zc); /* 0x0000..0x14eb */
            float blend2 = fog*fogDensity/(float)0x2000;
            if( blend2>0.0f )
            {
	            float red2   = (nthbyte( namcos22_gamma, 0x05 )/255.0f);
	            float green2 = (nthbyte( namcos22_gamma, 0x06 )/255.0f);
               float blue2  = (nthbyte( namcos22_gamma, 0x07 )/255.0f);
               if( blend2>1.0f )
               {
                  blend2 = 1.0f;
               }
               red = (red*blend+red2*blend2)/(blend+blend2);
               green = (green*blend+green2*blend2)/(blend+blend2);
               blue = (blue*blend+blue2*blend2)/(blend+blend2);
               blend = 1.0 - (1.0-blend)*(1.0-blend2);
            } /* blend2 */
         } /* fog>0 */
      } /* namcos22_czram */

      if( fadeTargetFlags&1 )
      { /* affects polygon layer */
         int fade  = nthbyte( namcos22_gamma, 0x19 );
         if( fade )
         {
	         float red2   = (nthbyte( namcos22_gamma, 0x16 )/255.0f);
	         float green2 = (nthbyte( namcos22_gamma, 0x17 )/255.0f);
            float blue2  = (nthbyte( namcos22_gamma, 0x18 )/255.0f);
            float blend2 = fade/255.0f;

            red = (red*blend+red2*blend2)/(blend+blend2);
            green = (green*blend+green2*blend2)/(blend+blend2);
            blue = (blue*blend+blue2*blend2)/(blend+blend2);
            blend = 1.0 - (1.0-blend)*(1.0-blend2);
         } /* fade */
      } /* fadeTargetFlags */
   } /* namcos22_gamma */

   p->red = red;
   p->green = green;
   p->blue = blue;
   p->blend = blend;
} /* ApplyFade */

void
namcos22_draw_direct_poly( const UINT16 *pSource )
{
	/**
    * 0x03a2 // 0x0fff zsortvalue24?
    * 0x0001 // 0x000f tpage?
    * 0xbd00 // color
    * 0x13a2 // flags
    *
    * 0x0100 0x009c // u,v
    * 0x0072 0xf307 // sx,sy
    * 0x602b 0x9f28 // i,zpos
    *
    * 0x00bf 0x0060 // u,v
    * 0x0040 0xf3ec // sx,sy
    * 0x602b 0xad48 // i,zpos
    *
    * 0x00fb 0x00ca // u,v
    * 0x0075 0xf205 // sx,sy
    * 0x602b 0x93e8 // i,zpos
    *
    * 0x00fb 0x00ca // u,v
    * 0x0075 0xf205 // sx,sy
    * 0x602b 0x93e8 // i,zpos
    */
   INT32 zsortvalue24 = pSource[0];
   int bank = (pSource[5]>>12)&0xf;
   /* pSource[1]? */
   unsigned color = pSource[2];
   INT32 flags = pSource[3]; /* ? */
   struct SceneNode *node = NewSceneNode(zsortvalue24,eSCENENODE_QUAD3D);
   int i;
   node->data.quad3d.flags = flags;
   node->data.quad3d.pTexture = GetTextureCache(bank);
   node->data.quad3d.color = (color>>8)&0x7f;
   pSource += 4;
   for( i=0; i<4; i++ )
   {
      Poly3dVertex *p = &node->data.quad3d.v[i];
      p->u = pSource[0]&0xfff;
      p->v = pSource[1]&0xfff;
      p->z = DspFloatToNativeFloat( (pSource[4]<<16) | pSource[5] );
      p->x = (INT16)pSource[2]*p->z;
      p->y = -(INT16)pSource[3]*p->z;
      ApplyFade( p, pSource[4]>>8 );
      pSource += 6;
   }
} /* namcos22_draw_direct_poly */

static int
Prepare3dTexture( void *pTilemapROM, void *pTextureROM )
{
    int i;
    if( pTilemapROM && pTextureROM )
    { /* following setup is Namco System 22 specific */
	      const UINT8 *pPackedTileAttr = 0x200000 + (UINT8 *)pTilemapROM;
	      UINT8 *pUnpackedTileAttr = auto_malloc(0x080000*2);
	      if( pUnpackedTileAttr )
      	{
       	   InitXYAttrToPixel();
   	      mpTextureTileMapAttr = pUnpackedTileAttr;
   	      for( i=0; i<0x80000; i++ )
   	      {
   	         *pUnpackedTileAttr++ = (*pPackedTileAttr)>>4;
   	         *pUnpackedTileAttr++ = (*pPackedTileAttr)&0xf;
   	         pPackedTileAttr++;
   	   }
   	   mpTextureTileMap16 = pTilemapROM;
         mpTextureTileData = pTextureROM;
   	   PatchTexture();
 	      return 0;
      }
   }
   return -1;
} /* Prepare3dTexture */

static void
GetSpriteTileData( int tileNumber, int color, unsigned char *pBuf, void *pUserParam )
{
   const gfx_element *gfx = Machine->gfx[0];
   const UINT8 *pPenData = gfx->gfxdata + (tileNumber%gfx->total_elements)*gfx->char_modulo;
	const pen_t *pPal = &gfx->colortable[gfx->color_granularity *(color%gfx->total_colors)];
   int x,y;
   for( y=0; y<32; y++ )
   {
      for( x=0; x<32; x++ )
      {
         int pen = *pPenData++;
         if( pen == 0xff )
         {
            *pBuf++ = 0x00;
            *pBuf++ = 0x00;
            *pBuf++ = 0x00;
            *pBuf++ = 0x00; /* transparent */
         }
         else
         {
            UINT32 rgb = pPal[pen];
            *pBuf++ = rgb>>16;
            *pBuf++ = (rgb>>8)&0xff;
            *pBuf++ = rgb&0xff;
            *pBuf++ = 0xff; /* opaque */
         }
      }
   }
} /* GetSpriteTileData */

static void
DrawSpritesHelper(
	mame_bitmap *bitmap,
	const rectangle *cliprect,
	const UINT32 *pSource,
	const UINT32 *pPal,
	int num_sprites,
	int deltax,
	int deltay )
{
   int i;
	for( i=0; i<num_sprites; i++ )
	{
      /*
         ----.-x--.----.----.----.----.----.---- hidden?
         ----.--x-.----.----.----.----.----.---- ?
         ----.---x.----.----.----.----.----.---- ?
         ----.----.xxxx.xxxx.xxxx.----.----.---- always 0xff0?
         ----.----.----.----.----.--x-.----.---- right justify
         ----.----.----.----.----.---x.----.---- bottom justify
         ----.----.----.----.----.----.x---.---- flipx
         ----.----.----.----.----.----.-xxx.---- numcols
         ----.----.----.----.----.----.----.x--- flipy
         ----.----.----.----.----.----.----.-xxx numrows
      */
		UINT32 attrs = pSource[2];
      if( (attrs&0x04000000)==0 )
		{ /* sprite is not hidden */
			INT32 zcoord = pPal[0];
         int color = pPal[1]>>16;
         //int cz = pPal[1]&0xffff;
			UINT32 xypos = pSource[0];
			UINT32 size = pSource[1];
			UINT32 code = pSource[3];
			int xpos = (xypos>>16)-deltax;
			int ypos = (xypos&0xffff)-deltay;
			int sizex = size>>16;
			int sizey = size&0xffff;
			int zoomx = (1<<16)*sizex/0x20;
			int zoomy = (1<<16)*sizey/0x20;
			int flipy = attrs&0x8;
			int numrows = attrs&0x7; /* 0000 0001 1111 1111 0000 0000 fccc frrr */
			int flipx = (attrs>>4)&0x8;
			int numcols = (attrs>>4)&0x7;
			int tile = code>>16;
         int translucency = (code&0xff00)>>8;

         if( numrows==0 )
         {
            numrows = 8;
         }
			if( flipy )
			{
				ypos += sizey*(numrows-1);
				sizey = -sizey;
			}

			if( numcols==0 )
         {
            numcols = 8;
         }
			if( flipx )
			{
				xpos += sizex*(numcols-1);
				sizex = -sizex;
			}

			if( attrs & 0x0200 )
			{ /* right justify */
				xpos -= ((zoomx*numcols*0x20)>>16)-1;
         }
			if( attrs & 0x0100 )
			{ /* bottom justify */
         	ypos -= ((zoomy*numrows*0x20)>>16)-1;
         }

         if( namcos22_gametype == NAMCOS22_ALPINE_RACER )
         { /* hack */
            xpos -= 48;
			   ypos -= 43;
         }

         {
            struct SceneNode *node = NewSceneNode(zcoord,eSCENENODE_SPRITE);
            node->data.sprite.tile = tile;
            node->data.sprite.color = color;
            node->data.sprite.flipx = flipx;
            node->data.sprite.flipy = flipy;
            node->data.sprite.numcols = numcols;
            node->data.sprite.numrows = numrows;
            node->data.sprite.xpos = xpos;
            node->data.sprite.ypos = ypos;
            node->data.sprite.sizex = sizex;
            node->data.sprite.sizey = sizey;
            node->data.sprite.translucency = translucency;
         }
  		} /* visible sprite */
		pSource += 4;
		pPal += 2;
   }
} /* DrawSpritesHelper */

static void
DrawSprites( mame_bitmap *bitmap, const rectangle *cliprect )
{
	/*
        0x980000:   00060000 00010000 02ff0000 000007ff
                             ^^^^                       num sprites

        0x980010:   00200020 000002ff 000007ff 00000000
                    ^^^^^^^^                            character size?
                             ^^^^                       delta xpos?
                                      ^^^^              delta ypos?

        0x980200:   000007ff 000007ff       delta xpos, delta ypos?
        0x980208:   000007ff 000007ff
        0x980210:   000007ff 000007ff
        0x980218:   000007ff 000007ff
        0x980220:   000007ff 000007ff
        0x980228:   000007ff 000007ff
        0x980230:   000007ff 000007ff
        0x980238:   000007ff 000007ff

        //time crisis
        00980200:  000007ff 000007ff 000007ff 032a0509
        00980210:  000007ff 000007ff 000007ff 000007ff
        00980220:  000007ff 000007ff 000007ff 000007ff
        00980230:  000007ff 000007ff 05000500 050a050a

        0x980400:   zoom table?
        0x980600:   zoom table?
            0000    0000.0000.0000.0000
            8000    1000.0000.0000.0000
            8080    1000.0000.1000.0000
            8880    1000.1000.1000.0000
            8888    1000.1000.1000.1000
            a888    1010.1000.1000.1000
            a8a8    1010.1000.1010.1000
            aaa8    1010.1010.1010.1000
            aaaa    1010.1010.1010.1010
            eaaa    1110.1010.1010.1010
            eaea    1110.1010.1110.1010
            eeea    1110.1110.1110.1010
            eeee    1110.1110.1110.1110
            feee    1111.1110.1110.1110
            fefe    1111.1110.1111.1110
            fffe    1111.1111.1111.1110
            ffff    1111.1111.1111.1111

        unknown table:
        0x980800:   0000 0001 0002 0003 ... 03ff
        
        eight words per sprite:
        0x984000:   010f 007b   xpos, ypos
        0x984004:   0020 0020   size x, size y
        0x984008:   00ff 0311   00ff, chr x;chr y;flip x;flip y
        0x98400c:   0001 0000   sprite code, translucency
        ...

        additional sorting/color data for sprite:
        0x9a0000:   C381 Z (sort)
        0x9a0004:   palette, C381 ZC (depth cueing)
        ...
    */
	int num_sprites = ((spriteram32[0x04/4]>>16)&0x3ff)+1;
	const UINT32 *pSource = &spriteram32[0x4000/4];
	const UINT32 *pPal = &spriteram32[0x20000/4];
   int deltax = spriteram32[0x14/4]>>16;
	int deltay = spriteram32[0x18/4]>>16;

   if( namcos22_gametype != NAMCOS22_AIR_COMBAT22 )
   {
      DrawSpritesHelper( bitmap, cliprect, pSource, pPal, num_sprites, deltax, deltay );
   }

	/* VICS RAM provides two additional banks */
	/*
        0x940000 -x------       sprite chip busy
        0x940018 xxxx----       clr.w   $940018.l

        0x940034 xxxxxxxx       0x3070b0f
        
        0x940040 xxxxxxxx       sprite attribute size
        0x940048 xxxxxxxx       sprite attribute list baseaddr
        0x940050 xxxxxxxx       sprite color size
        0x940058 xxxxxxxx       sprite color list baseaddr
        
        0x940060..0x94007c      set#2
*/

	num_sprites = (namcos22_vics_control[0x40/4]&0xffff)/0x10;
	if( num_sprites>=1 )
	{
		pSource = &namcos22_vics_data[(namcos22_vics_control[0x48/4]&0xffff)/4];
		pPal    = &namcos22_vics_data[(namcos22_vics_control[0x58/4]&0xffff)/4];
		DrawSpritesHelper( bitmap, cliprect, pSource, pPal, num_sprites, deltax, deltay );
	}

	num_sprites = (namcos22_vics_control[0x60/4]&0xffff)/0x10;
	if( num_sprites>=1 )
	{
		pSource = &namcos22_vics_data[(namcos22_vics_control[0x68/4]&0xffff)/4];
		pPal    = &namcos22_vics_data[(namcos22_vics_control[0x78/4]&0xffff)/4];
		DrawSpritesHelper( bitmap, cliprect, pSource, pPal, num_sprites, deltax, deltay );
	}
} /* DrawSprites */

static void
UpdatePaletteS( void ) /* for Super System22 - apply gamma correction and preliminary fader support */
{
	int i,j;

	int red   = nthbyte( namcos22_gamma, 0x16 );
	int green = nthbyte( namcos22_gamma, 0x17 );
	int blue  = nthbyte( namcos22_gamma, 0x18 );
	int fade  = nthbyte( namcos22_gamma, 0x19 );
	/* int flags = nthbyte( namcos22_gamma, 0x1a ); */

	for( i=0; i<NAMCOS22_PALETTE_SIZE/4; i++ )
	{
		if( dirtypal[i] )
		{
			for( j=0; j<4; j++ )
			{
				int which = i*4+j;
				int r = nthbyte(paletteram32,which+0x00000);
				int g = nthbyte(paletteram32,which+0x08000);
				int b = nthbyte(paletteram32,which+0x10000);

				if( fade && 0 )
				{ /**
                   * if flags&0x01 is set, fader affects polygon layer
                   * flags&0x02 and flags&0x04 are used to fade text/sprite layer
                   *
                   * for now, ignore flags and fade all palette entries
                   */
					r = (r*(0x100-fade)+red*fade)/256;
					g = (g*(0x100-fade)+green*fade)/256;
					b = (b*(0x100-fade)+blue*fade)/256;
				}

				/* map through gamma table (before or after fader?) */
				r = nthbyte( &namcos22_gamma[0x100/4], r );
				g = nthbyte( &namcos22_gamma[0x200/4], g );
				b = nthbyte( &namcos22_gamma[0x300/4], b );

				palette_set_color( which,r,g,b );
			}
			dirtypal[i] = 0;
		}
	}
} /* UpdatePaletteS */

static void
UpdatePalette( void ) /* for System22 - ignore gamma/fader effects for now */
{
	int i,j;
	for( i=0; i<NAMCOS22_PALETTE_SIZE/4; i++ )
	{
		if( dirtypal[i] )
		{
			for( j=0; j<4; j++ )
			{
				int which = i*4+j;
				int r = nthbyte(paletteram32,which+0x00000);
				int g = nthbyte(paletteram32,which+0x08000);
				int b = nthbyte(paletteram32,which+0x10000);
				palette_set_color( which,r,g,b );
			}
			dirtypal[i] = 0;
		}
	}
} /* UpdatePalette */

static void
GetTextTileData( int tileNumber, int color, unsigned char *pBuf, void *pUserParam )
{
   int palBase = namcos22_gamma?(nthbyte(namcos22_gamma,0x1b)*256):0x7f00;
   const gfx_element *gfx = Machine->gfx[NAMCOS22_ALPHA_GFX];
   const UINT8 *pPenData = gfx->gfxdata + (tileNumber%gfx->total_elements)*gfx->char_modulo;
	const pen_t *pPal = &gfx->colortable[gfx->color_granularity*(color%gfx->total_colors)];
   int x,y;
   for( y=0; y<16; y++ )
   {
      for( x=0; x<16; x++ )
      {
         int pen = *pPenData++;
         if( pen == 0xf )
         {
            *pBuf++ = 0x00;
            *pBuf++ = 0x00;
            *pBuf++ = 0x00;
            *pBuf++ = 0x00;
         }
         else
         {
            UINT32 rgb = pPal[pen+palBase];
            *pBuf++ = rgb>>16;
            *pBuf++ = (rgb>>8)&0xff;
            *pBuf++ = rgb&0xff;
            *pBuf++ = 0xff;
         }
      }
   }
} /* GetTextTileData */

static void
DrawTextLayer( mame_bitmap *bitmap, const rectangle *cliprect )
{
	unsigned i;
   INT32 dx = namcos22_tilemapattr?(namcos22_tilemapattr[0]>>16):0x35c;
   INT32 dy = namcos22_tilemapattr?(namcos22_tilemapattr[0]&0xffff):0;
   /* tilemap[1] == 0x006e0000
    * tilemap[2] == 0x01ff0000
    */

   if( cgsomethingisdirty )
	{
		for( i=0; i<NUM_CG_CHARS; i++ )
		{
			if( cgdirty[i] )
			{
				decodechar( Machine->gfx[NAMCOS22_ALPHA_GFX],i,(UINT8 *)namcos22_cgram,&cg_layout );
				cgdirty[i] = 0;
            poly3d_DirtyTileNumber( mpTextTileCache, i );
			}
		}
		cgsomethingisdirty = 0;
	}

   {
      int col,row;
      int tileIndex = 0;
      poly3d_Begin2d(mpTextTileCache);
      for( row=0; row<64; row++ )
      {
         for( col=0; col<64; col++ )
         {
            UINT16 data = nthword( namcos22_textram,tileIndex++ );
           	/**
             * xxxx------------ palette select
             * ----xx---------- flip
             * ------xxxxxxxxxx code
             */
            int flipy = data&0x800;
            int flipx = data&0x400;
            int tileNumber = data&0x3ff;
            int color = data>>12;
            int sx = (col*16 + 0x35c - dx/*+9*/)&0x3ff;
            int sy = (row*16 - dy/*-9*/)&0x3ff;
            if( sx>640 ) sx-= 0x400;
            if( sy>480 ) sy-= 0x400;
            poly3d_DrawTile( tileNumber, color, sx, sy, 16, 16,flipx,flipy );
         }
      }
   }
} /* DrawTextLayer */

/*********************************************************************************************/

#define MAX_FLAG 256
static INT32 mRegisteredFlag[MAX_FLAG];
static int mNumRegisteredFlags;
static int
RegisterFlag( INT32 flag )
{
   int i;
   for( i=0; i<mNumRegisteredFlags; i++ )
   {
      if( mRegisteredFlag[i]==flag ) goto L_Out;;
   }
   if( mNumRegisteredFlags<MAX_FLAG )
   {
      mRegisteredFlag[i] = flag;
      printf( "new flag=0x%06x\n", flag );
      mNumRegisteredFlags++;
   }
   else
   {
      exit(1);
   }
L_Out:
   {
      UINT32 t = 0;
      if( code_pressed(KEYCODE_Q) ) t|=(1<<0);
      if( code_pressed(KEYCODE_W) ) t|=(1<<1);
      if( code_pressed(KEYCODE_E) ) t|=(1<<2);
      if( code_pressed(KEYCODE_R) ) t|=(1<<3);
      if( code_pressed(KEYCODE_T) ) t|=(1<<4);
      if( code_pressed(KEYCODE_Y) ) t|=(1<<5);
      if( code_pressed(KEYCODE_U) ) t|=(1<<6);
      if( code_pressed(KEYCODE_I) ) t|=(1<<7);
      if( code_pressed(KEYCODE_O) ) t|=(1<<8);
      if( code_pressed(KEYCODE_A) ) t|=(1<<9);
      if( code_pressed(KEYCODE_S) ) t|=(1<<10);
      if( code_pressed(KEYCODE_D) ) t|=(1<<11);
      if( code_pressed(KEYCODE_F) ) t|=(1<<12);
      if( code_pressed(KEYCODE_G) ) t|=(1<<13);
      if( code_pressed(KEYCODE_H) ) t|=(1<<14);
      if( code_pressed(KEYCODE_J) ) t|=(1<<15);
      if( code_pressed(KEYCODE_K) ) t|=(1<<16);
      if( code_pressed(KEYCODE_L) ) t|=(1<<17);
      if( code_pressed(KEYCODE_Z) ) t|=(1<<18);
      if( code_pressed(KEYCODE_X) ) t|=(1<<19);
      if( code_pressed(KEYCODE_C) ) t|=(1<<20);
      if( code_pressed(KEYCODE_V) ) t|=(1<<21);
      if( code_pressed(KEYCODE_B) ) t|=(1<<22);
      if( code_pressed(KEYCODE_N) ) t|=(1<<23);
      if( code_pressed(KEYCODE_M) ) t|=(1<<24);
      if( t==0 || ((t>>i)&1) )
      {
         static UINT32 tt = 0;
         if( t && tt!=t )
         {
            printf( "0x%06x\n", mRegisteredFlag[i] );
         }
         tt = t;
         return 1;
      }
   }
   return 0;
}

static int
Cap( int val, int minval, int maxval )
{
   if( val<minval )
   {
      val = minval;
   }
   else if( val>maxval )
   {
      val = maxval;
   }
   return val;
}

#define LSB21 (0x1fffff)
#define LSB18 (0x03ffff)

static INT32
Signed18( UINT32 value )
{
   INT32 offset = value&LSB18;
   if( offset&0x20000 )
   { /* sign extend */
		offset |= ~LSB18;
   }
   return offset;
}

/**
 * @brief render a single quad
 *
 * @param flags
 *     -1.----.-1--.--1- always set
 *     x-.----.----.---- priority over tilemap
 *     --.--xx.----.---- representative z algorithm
 *     --.----.--x-.---- backface cull enable
 *     --.----.----.---x fog enable?
 *
 * @param color
 *      -------- xxxxxxxx unused?
 *      -xxxxxxx -------- palette select
 *      x------- -------- ?
 *
 * @param polygonShiftValue22
 *    0x1fbd0 - sky+sea
 *    0x0c350 - mountins
 *    0x09c40 - boats, surf, road, buildings
 *    0x07350 - guardrail
 *    0x061a8 - red car
 */
static void
BlitQuadHelper(
		mame_bitmap *pBitmap,
		unsigned color,
		unsigned addr,
		float m[4][4],
		INT32 polygonShiftValue22, /* 22 bits */
		INT32 flags )
{
   int absolutePriority = mAbsolutePriority;
   UINT32 zsortvalue24;
   float zmin = 0.0f;
   float zmax = 0.0f;
	Poly3dVertex v[4];
	int i;
   int bBackFace = 0;
   float k;

	for( i=0; i<4; i++ )
	{
		Poly3dVertex *pVerTex = &v[i];
		pVerTex->x = GetPolyData(  8+i*3+addr );
		pVerTex->y = GetPolyData(  9+i*3+addr );
		pVerTex->z = GetPolyData( 10+i*3+addr );
		TransformPoint( &pVerTex->x, &pVerTex->y, &pVerTex->z, m );
	} /* for( i=0; i<4; i++ ) */

   k = (v[2].x*((v[0].z*v[1].y)-(v[0].y*v[1].z)))+
       (v[2].y*((v[0].x*v[1].z)-(v[0].z*v[1].x)))+
		 (v[2].z*((v[0].y*v[1].x)-(v[0].x*v[1].y)));
   
//   k = (v[0].x*((v[2].z*v[3].y)-(v[2].y*v[3].z)))+
//		 (v[0].y*((v[2].x*v[3].z)-(v[2].z*v[3].x)))+
//		 (v[0].z*((v[2].y*v[3].x)-(v[2].x*v[3].y)));
   
   if( k >= 0 )
   {
      bBackFace = 1;
   }
   k = -k;

   if( bBackFace && (flags&0x0020) )
	{ /* backface cull one-sided polygons */
			return;
	}

   for( i=0; i<4; i++ )
	{
		Poly3dVertex *pVerTex = &v[i];
      int bri;

		pVerTex->u = GetPolyData(  0+i*2+addr );
		pVerTex->v = GetPolyData(  1+i*2+addr );

		if( i==0 || pVerTex->z > zmax ) zmax = pVerTex->z;
		if( i==0 || pVerTex->z < zmin ) zmin = pVerTex->z;

      if( mLitSurfaceCount )
      {
      	bri = mLitSurfaceInfo[mLitSurfaceIndex%mLitSurfaceCount];
         if( mSurfaceNormalFormat == 0x6666 )
         {
            if( i==3 )
            {
               mLitSurfaceIndex++;
            }
         }
         else if( mSurfaceNormalFormat == 0x4000 )
         {
            mLitSurfaceIndex++;
         }
         else
         {
            logerror( "unknown normal format: 0x%x\n", mSurfaceNormalFormat );
         }
      } /* pLitSurfaceInfo */
      else
      {
         bri = (GetPolyData(i+addr)>>16)&0xff;
      }
      ApplyFade( pVerTex, bri );
	} /* for( i=0; i<4; i++ ) */

   if( zmin<0.0f ) zmin = 0.0f;
   if( zmax<0.0f ) zmax = 0.0f;

   switch( (flags&0x0f00)>>8 )
   {
   case 0:
       zsortvalue24 = (INT32)zmin;
       break;

   case 1:
       zsortvalue24 = (INT32)zmax;
       break;

   case 2:
   default:
       zsortvalue24 = (INT32)((zmin+zmax)/2.0f);
       break;
   }

//   if( !RegisterFlag(polygonShiftValue22) )
//   {
//      return;
//   }

   /* relative: representative z + shift values
    * 1x.xxxx.xxxxxxxx.xxxxxxxx fixed z value
    * 0x.xx--.--------.-------- absolute priority shift
    * 0-.--xx.xxxxxxxx.xxxxxxxx z-representative value shift
    */
   if( polygonShiftValue22 & 0x200000 )
   {
      zsortvalue24 = polygonShiftValue22 & LSB21;
   }
   else
   {
      zsortvalue24 += /* k* */Signed18( polygonShiftValue22 );
      absolutePriority += (polygonShiftValue22&0x1c0000)>>18;
   }
   if( mObjectShiftValue22 & 0x200000 )
   {
      zsortvalue24 = mObjectShiftValue22 & LSB21;
   }
   else
   {
      zsortvalue24 += /* k* */Signed18( mObjectShiftValue22 );
      absolutePriority += (mObjectShiftValue22&0x1c0000)>>18;
   }
   absolutePriority &= 7;
   zsortvalue24 = Cap(zsortvalue24,0,0x1fffff);
   zsortvalue24 |= (absolutePriority<<21);

   {
      struct SceneNode *node = NewSceneNode(zsortvalue24,eSCENENODE_QUAD3D);
      int bank = (v[0].v>>12)&0xf;
      node->data.quad3d.pTexture = GetTextureCache(bank);
      node->data.quad3d.color = (color>>8)&0x7f;
      node->data.quad3d.flags = flags;
      for( i=0; i<4; i++ )
      {
         Poly3dVertex *p = &node->data.quad3d.v[i];
         p->x = v[i].x*mCamera.zoom;
         p->y = v[i].y*mCamera.zoom;
         p->z = v[i].z;
         p->u = v[i].u&0xfff;
         p->v = v[i].v&0xfff;
         p->red   = v[i].red;
         p->green = v[i].green;
         p->blue  = v[i].blue;
         p->blend = v[i].blend;
      }
      node->data.quad3d.vx = mCamera.vx;
      node->data.quad3d.vy = mCamera.vy;
      node->data.quad3d.vw = mCamera.vw;
      node->data.quad3d.vh = mCamera.vh;
   }
} /* BlitQuadHelper */

static void
RegisterNormals( INT32 addr, float m[4][4] )
{
   int i;
   for( i=0; i<4; i++ )
   {
         float nx = DSP_FIXED_TO_FLOAT(GetPolyData(addr+i*3+0));
		   float ny = DSP_FIXED_TO_FLOAT(GetPolyData(addr+i*3+1));
		   float nz = DSP_FIXED_TO_FLOAT(GetPolyData(addr+i*3+2));
			float dotproduct;

			/* transform normal vector */
			TransformNormal( &nx, &ny, &nz, m );
			dotproduct = nx*mCamera.lx + ny*mCamera.ly + nz*mCamera.lz;
			if( dotproduct<0.0f )
         {
            dotproduct = 0.0f;
         }
         mLitSurfaceInfo[mLitSurfaceCount++] = mCamera.ambient + mCamera.power*dotproduct;
   }
} /* RegisterNormals */

static void
BlitQuads( mame_bitmap *pBitmap, INT32 addr, float m[4][4], INT32 base )
{
   INT32 numAdditionalNormals = 0;
	INT32 size = GetPolyData(addr++);
	INT32 finish = addr + (size&0xff);
	INT32 flags;
	INT32 color;
	INT32 bias;
   
	while( addr<finish )
	{
		size = GetPolyData(addr++);
		size &= 0xff;
		switch( size )
		{
		case 0x17:
			/**
             * word 0: opcode (8a24c0)
             * word 1: flags
             * word 2: color
             */
			flags = GetPolyData(addr+1);
			color = GetPolyData(addr+2);
			bias = 0;
			BlitQuadHelper( pBitmap,color,addr+3,m,bias,flags );
			break;

		case 0x18:
			/**
          * word 0: opcode (0b3480 for first N-1 quads or 8b3480 for final quad in primitive)
          * word 1: flags
          * word 2: color
          * word 3: depth bias
          */
			flags = GetPolyData(addr+1);
			color = GetPolyData(addr+2);
			bias  = GetPolyData(addr+3);
			BlitQuadHelper( pBitmap,color,addr+4,m,bias,flags );
			break;

		case 0x10: /* vertex lighting */
			/*
            333401 (opcode)
                000000  [count] [type]
                000000  000000  007fff // normal vector
                000000  000000  007fff // normal vector
                000000  000000  007fff // normal vector
                000000  000000  007fff // normal vector
            */
         numAdditionalNormals = GetPolyData(addr+2);
         mSurfaceNormalFormat = GetPolyData(addr+3);
         mLitSurfaceCount = 0;
         mLitSurfaceIndex = 0;
         RegisterNormals( addr+4, m );
			break;

		case 0x0d: /* additional normals */
			/*
            300401 (opcode)
                007b09 ffdd04 0004c2
                007a08 ffd968 0001c1
                ff8354 ffe401 000790
                ff84f7 ffdd04 0004c2
            */
         RegisterNormals( addr+1, m );
         break;

		default:
         logerror( "BlitQuads; unk opcode=0x%x\n", size );
			break;
		}
		addr += size;
	}
} /* BlitQuads */

static void
BlitPolyObject( mame_bitmap *pBitmap, int code, float M[4][4] )
{
	unsigned addr1 = GetPolyData(code);
   mLitSurfaceCount = 0;
   mLitSurfaceIndex = 0;
	for(;;)
	{
		INT32 addr2 = GetPolyData(addr1++);
		if( addr2<0 )
		{
			break;
		}
		BlitQuads( pBitmap, addr2, M, code );
	}
} /* BlitPolyObject */

/*******************************************************************************/

READ32_HANDLER( namcos22_dspram_r )
{
	return namcos22_polygonram[offset];
}

WRITE32_HANDLER( namcos22_dspram_w )
{
	COMBINE_DATA( &namcos22_polygonram[offset] );
	namcos22_UploadCodeToDSP();
}

/*******************************************************************************/

/**
 * master DSP can write directly to render device via port 0xc.
 * This is used for "direct drawn" polygons, and "direct draw from point rom"
 * feature - both opcodes exist in Ridge Racer's display-list processing
 *
 * record format:
 *  header (3 words)
 *      polygonShiftValue22
 *      color
 *      flags
 *
 *  per-vertex data (4*6 words)
 *      u,v
 *      sx,sy
 *      intensity;z.exponent
 *      z.mantissa
 *
 * master DSP can specify 3d objects indirectly (along with view transforms),
 * via the "transmit" PDP opcode.  the "render device" sends quad data to the slave DSP
 * viewspace clipping and projection
 *
 * most "3d object" references are 0x45 and greater.  references less than 0x45 are "special"
 * commands, using a similar point rom format.  the point rom header may point to point ram.
 *
 * slave DSP reads records via port 4
 * its primary purpose is applying lighting calculations
 * the slave DSP forwards draw commands to a "draw device"
 */

/*******************************************************************************/

/**
 * 0xfffd
 * 0x0: transform
 * 0x1
 * 0x2
 * 0x5: transform
 * >=0x45: draw primitive
 */
static void
HandleBB0003( const INT32 *pSource )
{
   /*
        bb0003 or 3b0003

        14.00c8            light.ambient     light.power
        01.0000            ?                 light.dx
        06.5a82            window priority   light.dy
        00.a57e            ?                 light.dz

        c8.0081            vx=200,vy=129
        29.6092            zoom = 772.5625
        1e.95f8 1e.95f8            0.5858154296875   0.5858154296875 // 452
        1e.b079 1e.b079            0.6893463134765   0.6893463134765 // 532
        29.58e8                   711.25 (border? see time crisis)

        7ffe 0000 0000
        0000 7ffe 0000
        0000 0000 7ffe
    */
	mCamera.ambient = pSource[0x1]>>16;
	mCamera.power   = pSource[0x1]&0xffff;

	mCamera.lx       = DSP_FIXED_TO_FLOAT(pSource[0x2]);
	mCamera.ly       = DSP_FIXED_TO_FLOAT(pSource[0x3]);
	mCamera.lz       = DSP_FIXED_TO_FLOAT(pSource[0x4]);
	
   mAbsolutePriority = pSource[0x3]>>16;
   mCamera.vx      = (INT16)(pSource[5]>>16);
   mCamera.vy      = (INT16)pSource[5];
   mCamera.zoom    = DspFloatToNativeFloat(pSource[6]);
   mCamera.vw      = DspFloatToNativeFloat(pSource[7])*mCamera.zoom;
   mCamera.vh      = DspFloatToNativeFloat(pSource[9])*mCamera.zoom;

	mViewMatrix[0][0] = DSP_FIXED_TO_FLOAT(pSource[0x0c]);
	mViewMatrix[1][0] = DSP_FIXED_TO_FLOAT(pSource[0x0d]);
	mViewMatrix[2][0] = DSP_FIXED_TO_FLOAT(pSource[0x0e]);

	mViewMatrix[0][1] = DSP_FIXED_TO_FLOAT(pSource[0x0f]);
	mViewMatrix[1][1] = DSP_FIXED_TO_FLOAT(pSource[0x10]);
	mViewMatrix[2][1] = DSP_FIXED_TO_FLOAT(pSource[0x11]);

	mViewMatrix[0][2] = DSP_FIXED_TO_FLOAT(pSource[0x12]);
	mViewMatrix[1][2] = DSP_FIXED_TO_FLOAT(pSource[0x13]);
	mViewMatrix[2][2] = DSP_FIXED_TO_FLOAT(pSource[0x14]);

	TransformNormal( &mCamera.lx, &mCamera.ly, &mCamera.lz, mViewMatrix );
} /* HandleBB0003 */

static void
Handle200002( mame_bitmap *bitmap, const INT32 *pSource )
{
	if( mPrimitiveID>=0x45 )
	{
		float m[4][4]; /* row major */

		matrix3d_Identity( m );

		m[0][0] = DSP_FIXED_TO_FLOAT(pSource[0x1]);
		m[1][0] = DSP_FIXED_TO_FLOAT(pSource[0x2]);
		m[2][0] = DSP_FIXED_TO_FLOAT(pSource[0x3]);

		m[0][1] = DSP_FIXED_TO_FLOAT(pSource[0x4]);
		m[1][1] = DSP_FIXED_TO_FLOAT(pSource[0x5]);
		m[2][1] = DSP_FIXED_TO_FLOAT(pSource[0x6]);

		m[0][2] = DSP_FIXED_TO_FLOAT(pSource[0x7]);
		m[1][2] = DSP_FIXED_TO_FLOAT(pSource[0x8]);
		m[2][2] = DSP_FIXED_TO_FLOAT(pSource[0x9]);

		m[3][0] = pSource[0xa]; /* xpos */
		m[3][1] = pSource[0xb]; /* ypos */
		m[3][2] = pSource[0xc]; /* zpos */

      matrix3d_Multiply( m, mViewMatrix );
		BlitPolyObject( bitmap, mPrimitiveID, m );
	}
   else if( mPrimitiveID !=0 && mPrimitiveID !=2 )
   {
      logerror( "Handle200002:unk code=0x%x\n", mPrimitiveID );
   }
} /* Handle200002 */

static void
Handle300000( const INT32 *pSource )
{ /* set view transform */
	mViewMatrix[0][0] = DSP_FIXED_TO_FLOAT(pSource[1]);
	mViewMatrix[1][0] = DSP_FIXED_TO_FLOAT(pSource[2]);
	mViewMatrix[2][0] = DSP_FIXED_TO_FLOAT(pSource[3]);

	mViewMatrix[0][1] = DSP_FIXED_TO_FLOAT(pSource[4]);
	mViewMatrix[1][1] = DSP_FIXED_TO_FLOAT(pSource[5]);
	mViewMatrix[2][1] = DSP_FIXED_TO_FLOAT(pSource[6]);

	mViewMatrix[0][2] = DSP_FIXED_TO_FLOAT(pSource[7]);
	mViewMatrix[1][2] = DSP_FIXED_TO_FLOAT(pSource[8]);
	mViewMatrix[2][2] = DSP_FIXED_TO_FLOAT(pSource[9]);
} /* Handle300000 */

static void
Handle233002( const INT32 *pSource )
{ /* set modal rendering options */
   /*
    00233002
       00000000 // zc adjust?
       0003dd00 // z bias adjust
       001fffff // far plane?
       00007fff 00000000 00000000
       00000000 00007fff 00000000
       00000000 00000000 00007fff
       00000000 00000000 00000000
   */
   mObjectShiftValue22 = pSource[2];
} /* Handle233002 */

static void
SimulateSlaveDSP( mame_bitmap *bitmap )
{
	const INT32 *pSource = 0x300 + (INT32 *)namcos22_polygonram;
	INT16 len;

	matrix3d_Identity( mViewMatrix );

	if( mbSuperSystem22 )
	{
		pSource += 4; /* FFFE 0400 */
	}
	else
	{
		pSource--;
	}

	for(;;)
	{
		INT16 marker, next;
		mPrimitiveID = *pSource++;
		len  = (INT16)*pSource++;

		switch( len )
		{
		case 0x15:
			HandleBB0003( pSource ); /* define viewport */
			break;

		case 0x10:
			Handle233002( pSource ); /* set modal rendering options */
			break;

		case 0x0a:
			Handle300000( pSource ); /* modify view transform */
			break;

		case 0x0d:
			Handle200002( bitmap, pSource ); /* render primitive */
			break;

		default:
         logerror( "unk 3d data(%d)!", len );
         {
            int i;
            for( i=0; i<len; i++ )
            {
               logerror( " %06x", pSource[i]&0xffffff );
            }
            logerror( "\n" );
         }
			return;
		}

		/* hackery! commands should be streamed, not parsed here */
		pSource += len;
		marker = (INT16)*pSource++; /* always 0xffff */
		next   = (INT16)*pSource++; /* link to next command */
		if( (next&0x7fff) != (pSource - (INT32 *)namcos22_polygonram) )
		{ /* end of list */
			break;
		}
	} /* for(;;) */
} /* SimulateSlaveDSP */

static void
DrawPolygons( mame_bitmap *bitmap )
{
	if( mbDSPisActive )
	{
		SimulateSlaveDSP( bitmap );
	}
} /* DrawPolygons */

void
namcos22_dsp_enable( void )
{
	mbDSPisActive = 1;
}

/*********************************************************************************************/

READ32_HANDLER( namcos22_cgram_r )
{
	return namcos22_cgram[offset];
}

WRITE32_HANDLER( namcos22_cgram_w )
{
	COMBINE_DATA( &namcos22_cgram[offset] );
	cgdirty[offset/32] = 1;
	cgsomethingisdirty = 1;
}

READ32_HANDLER( namcos22_gamma_r )
{
	return namcos22_gamma[offset];
}

/*
    +0x0002.w   Fader Enable(?) (0: disabled)
    +0x0011.w   Display Fader (R) (0x0100 = 1.0)
    +0x0013.w   Display Fader (G) (0x0100 = 1.0)
    +0x0015.w   Display Fader (B) (0x0100 = 1.0)
    +0x0100.b   Fog1 Color (R) (world fogging)
    +0x0101.b   Fog2 Color (R) (used for heating of brake-disc on RV1)
    +0x0180.b   Fog1 Color (G)
    +0x0181.b   Fog2 Color (G)
    +0x0200.b   Fog1 Color (B)
    +0x0201.b   Fog2 Color (B)
*/

/**
 * 0x800000: 08380000
 *
 * 0x810000: 0004 0004 0004 0004 4444
 *           ^^^^ ^^^^ ^^^^ ^^^^      fog thickness? 0 if disabled
 *
 * 0x810200..0x8103ff: 0x100 words (depth cueing table)
 *      0000 0015 002a 003f 0054 0069 007e 0093 00a8 00bd 00d2 00e7 00fc 0111 0126 013b
 *
 * 0x810400..0x810403: (air combat22?)
 * 0x820000..0x8202ff: ?
 *
 * 0x824000..0x8243ff: gamma
 *   Prop Cycle: start of stage#2
 *   ffffff00 00ffffff 0000007f 00000000 0000ff00 0f00ffff ff00017f 00010007 00000001
 *
 *   Prop Cycle: submerged in swamp
 *   ffffff00 0003ffae 0000007f 00000000 0000ff00 0f00ffff ff00017f 00010007 00000001
 *
 * 0x860000..0x860007: ?
 */
WRITE32_HANDLER( namcos22_gamma_w )
{
	UINT32 old = namcos22_gamma[offset];
	COMBINE_DATA( &namcos22_gamma[offset] );
	if( old!=namcos22_gamma[offset] )
	{
		memset( dirtypal, 1, NAMCOS22_PALETTE_SIZE/4 );
	}
	/**
     * 824000: ffffff00 00ffffff 0000007f 00000000
     *                    ^^^^^^                    RGB  (fog color)
     *
     * 824010: 0000ff00 0f00RRGG BBII017f 00010007
     *                      ^^^^ ^^                 RGB(fade)
     *                             ^^               fade (zero for none, 0xff for max)
     *                               ^^             flags; fader targer
     *                                                     1: affects polygon layer
     *                                                     2: affects text(?)
     *                                                     4: affects sprites(?)
     *                                 ^^           tilemap palette base
     *
     * 824020: 00000001 00000000 00000000 00000000
     *
     * 824100: 00 05 0a 0f 13 17 1a 1e ... (red)
     * 824200: 00 05 0a 0f 13 17 1a 1e ... (green)
     * 824300: 00 05 0a 0f 13 17 1a 1e ... (blue)
     */
}

READ32_HANDLER( namcos22_paletteram_r )
{
	return paletteram32[offset];
}

WRITE32_HANDLER( namcos22_paletteram_w )
{
	COMBINE_DATA( &paletteram32[offset] );
	dirtypal[offset&(0x7fff/4)] = 1;
}

READ32_HANDLER( namcos22_textram_r )
{
	return namcos22_textram[offset];
}

WRITE32_HANDLER( namcos22_textram_w )
{
	COMBINE_DATA( &namcos22_textram[offset] );
}

static int
video_start_common( void )
{
	mbDSPisActive = 0;
	memset( namcos22_polygonram, 0xcc, 0x20000 );
/**
 * +0x00 0  always 0xf?
 * +0x04 1  sin(x) world-view matrix
 * +0x08 2  cos(x)
 * +0x0c 3  sin(y)
 * +0x10 4  cos(y)
 * +0x14 5  sin(z)
 * +0x18 6  cos(z)
 * +0x1c 7  ROLT? always 0x0002?
 * +0x20 8  light power
 * +0x24 9  light ambient
 * +0x28 10 light vector(x)
 * +0x2c 11 light vector(y)
 * +0x30 12 light vector(z)
 * +0x34 13 always 0x0002?
 * +0x38 14 field of view angle (fovx) in degrees
 * +0x3c 15 viewport width
 * +0x40 16 viewport height
 * +0x44 17
 * +0x48 18
 * +0x4c 19
 * +0x50 20 priority (top 7 > .. > 0 behind)
 * +0x54 21 viewport center x
 * +0x58 22 viewport center y
 * +0x5c 23 sin(x) ?
 * +0x60 24 cos(x)
 * +0x64 25 sin(y)
 * +0x68 26 cos(y)
 * +0x6c 27 sin(z)
 * +0x70 28 cos(z)
 * +0x74 29 flags (axis flipping?) 0004, 0003
 * +0x78 30 6630, 0001
 * +0x7c 31 7f02
 */
	if( Prepare3dTexture(
		memory_region(REGION_GFX3), /* texture tilemap */
		memory_region(REGION_GFX2)	/* texture tiles */
	) == 0 )
	{
		gfx_element *pGfx = decodegfx( (UINT8 *)namcos22_cgram,&cg_layout );
		if( pGfx )
		{
			Machine->gfx[NAMCOS22_ALPHA_GFX] = pGfx;
			pGfx->colortable = Machine->remapped_colortable;
			pGfx->total_colors = NAMCOS22_PALETTE_SIZE/16;
			dirtypal = auto_malloc(NAMCOS22_PALETTE_SIZE/4);
			if( dirtypal )
			{
				cgdirty = auto_malloc( 0x400 );
				if( cgdirty )
				{
					mPtRomSize = memory_region_length(REGION_GFX4)/3;
					mpPolyL = memory_region(REGION_GFX4);
					mpPolyM = mpPolyL + mPtRomSize;
					mpPolyH = mpPolyM + mPtRomSize;
					return 0; /* no error */
				}
			}
		}
	}
	return -1; /* error */
}

VIDEO_START( namcos22 )
{
   mbSuperSystem22 = 0;
   namcos22_gamma = NULL;
   mpTextTileCache = poly3d_CreateTileCache( 16,16,GetTextTileData, NULL,0,0, 1 );
   return video_start_common();
}

VIDEO_START( namcos22s )
{
   mbSuperSystem22 = 1;
   mpSpriteTileCache = poly3d_CreateTileCache( 32,32,GetSpriteTileData,NULL,0,0, 1 );
   mpTextTileCache = poly3d_CreateTileCache( 16,16,GetTextTileData, NULL,0,0, 1 );
   return video_start_common();
}

static void
Dump( FILE *f, unsigned addr1, unsigned addr2, const char *name )
{
   unsigned addr;
   fprintf( f, "%s:\n", name );
   for( addr=addr1; addr<=addr2; addr+=16 )
   {
      unsigned char data[16];
      int bHasNonZero = 0;
      int i;
      for( i=0; i<16; i++ )
      {
         data[i] = cpunum_read_byte( 0, addr+i );
         if( data[i] )
         {
            bHasNonZero = 1;
         }
      }
      if( bHasNonZero )
      {
         fprintf( f,"%08x:", addr );
         for( i=0; i<16; i++ )
         {
            if( (i&0x03)==0 )
            {
               fprintf( f, " " );
            }
            fprintf( f, "%02x", data[i] );
         }
         fprintf( f, "\n" );
      }
   }
   fprintf( f, "\n" );
}

VIDEO_UPDATE( namcos22s )
{
	int beamx,beamy;
//	cpunum_set_input_line(1, TMS32025_INT0, HOLD_LINE);
   poly3d_Begin();
	if( namcos22_gametype == NAMCOS22_TIME_CRISIS )
	{
		UpdatePalette();
	}
	else
	{
		UpdatePaletteS();
	}
   DrawPolygons( bitmap );
   DrawSprites( bitmap, cliprect );
   RenderScene(0);
   DrawTextLayer( bitmap, cliprect );
   RenderScene(1);
   poly3d_End();

   if( code_pressed(KEYCODE_D) )
   {
      FILE *f = fopen( "dump.txt", "wb" );
      if( f )
      {
      	Dump(f,0x800000, 0x80000f, "unk1" ); // 08380000
         
         Dump(f,0x940000, 0x94007f, "vics_control");
         Dump(f,0x900000, 0x90ffff, "vics_data");
         Dump(f,0x980000, 0x9affff, "sprite374" );

	      Dump(f,0x810000, 0x81000f, "cz attr" );
         //00810000:  00000000 00000000 44440000 00000000 // solitar
         //00810000:  00000000 00000000 75550000 00e40000 // normal
         //00810000:  7fff8000 7fff8000 75550000 00e40000 // offset
         //00810000:  00000000 00000000 31110000 00e40000 // off
         //00810000:  00040004 00040004 44440000 00000000 // out pool
         //00810000:  00a400a4 00a400a4 44440000 00000000 // in pool
         //00810000:  ff80ff80 ff80ff80 44440000 00000000 // ending
         //00810000:  ff80ff80 ff80ff80 00000000 00000000 // hs entry
         //00810000:  ff01ff01 00000000 00000000 00e40000 // alpine racer
         Dump(f,0x810200, 0x8103ff, "cz_ram");// 256 words; depth cueing

         Dump(f,0x820000, 0x8202ff, "unk3" );
         Dump(f,0x8a0000, 0x8a000f, "tilemap_attr");
	      Dump(f,0x824000, 0x8243ff, "gamma"); 
         Dump(f,0x880000, 0x89ffff, "text_ram");
         Dump(f,0xc00000, 0xc3ffff, "polygonram");

         fclose( f );
      }
      while( code_pressed(KEYCODE_D) ){} 
   }
   return;

	if( namcos22_gametype == NAMCOS22_TIME_CRISIS )
	{
		beamx = ((readinputport(1))*640)/256;
		beamy = ((readinputport(2))*480)/256;
		draw_crosshair( bitmap, beamx, beamy, cliprect, 0 );
	}
}

VIDEO_UPDATE( namcos22 )
{
//	cpunum_set_input_line(1, TMS32025_INT0, HOLD_LINE);
   poly3d_Begin();
	UpdatePalette();
   DrawPolygons( bitmap );
   RenderScene(0);
	DrawTextLayer( bitmap, cliprect );
   RenderScene(1);
   poly3d_End();
}

WRITE16_HANDLER( namcos22_dspram16_bank_w )
{
	COMBINE_DATA( &namcos22_dspram_bank );
}

READ16_HANDLER( namcos22_dspram16_r )
{
	UINT32 value = namcos22_polygonram[offset];
	switch( namcos22_dspram_bank )
	{
	case 0:
		value &= 0xffff;
		break;

	case 1:
		value>>=16;
		break;

	case 2:
		mUpperWordLatch = value>>16;
		value &= 0xffff;
		break;

	default:
		break;
	}
	return (UINT16)value;
} /* namcos22_dspram16_r */

WRITE16_HANDLER( namcos22_dspram16_w )
{
	UINT32 value = namcos22_polygonram[offset];
	UINT16 lo = value&0xffff;
	UINT16 hi = value>>16;
	switch( namcos22_dspram_bank )
	{
	case 0:
		COMBINE_DATA( &lo );
		break;

	case 1:
		COMBINE_DATA( &hi );
		break;

	case 2:
		COMBINE_DATA( &lo );
		hi = mUpperWordLatch;
		break;

	default:
		break;
	}
	namcos22_polygonram[offset] = (hi<<16)|lo;
} /* namcos22_dspram16_w */
