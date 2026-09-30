// ================================================================================================
//  ImageLoader - A RASTER IS A SOURCE BY BEING A FILE: GeoTIFF, PNG and JPEG through WIC (the
//  Windows Imaging Component), behind the FieldLoader seam (core/FieldLoader.h) its banner names:
//  registry.Register("tif", ...).
//
//  THE PIXELS are WIC's. The engine writes no TIFF decoder: what WIC cannot decode (a compression,
//  a sample layout) refuses THAT file, by name, with WIC's reason.
//
//  WHERE, in the file's own words first (GeoRef.h's EMBEDDED class): a GeoTIFF's ModelPixelScale
//  (33550) with ModelTiepoint (33922), or its ModelTransformation (34264); its GeoKeyDirectory
//  (34735) for the CRS and the raster type; GDAL's nodata (42113) -- each read through WIC's
//  metadata query reader. A PNG, a JPEG, or a TIFF without the tags: its world file (.pgw, .jgw,
//  .tfw, .wld) for the affine and its .prj's EPSG for the CRS. DECLARED (the scene entry's `crs`)
//  only where the file carries none. A GeoTIFF's OVERVIEWS are not read: WIC's TIFF decoder does
//  not show a reduced-resolution IFD as a frame (MEASURED 2026-09-29: a file of two IFDs, the
//  second NewSubfileType 1, decodes as one frame), and reaching one would mean walking the IFDs by
//  hand. The consumer builds its own chain (compose/RasterFileSource.cpp).
//
//  THE PROJECTIONS come from the code and are never guessed: EPSG 4326; 3857; UTM north and south
//  on WGS84 (326zz, 327zz), north on NAD83 (269zz) and NAD83(2011) (6330 to 6348), the zone read
//  off the code. Any other code is refused by name -- a code the loader does not know is not a
//  default zone. NAD83 is taken as WGS84 (~1 m), as Projections.h already takes it.
//
//  THE GEOREF IT GIVES: origin = the OUTER corner of texel (0, 0), scale signed (scaleY < 0: row 0
//  north), `centers` true -- a texel's sample stands at its centre, origin + scale (i + 0.5). A
//  GeoTIFF of PixelIsArea names that corner; PixelIsPoint and every world file name the CENTRE of
//  texel (0, 0), and the half texel is taken back here, once (priors 7). No row is flipped: the
//  sign of scaleY carries the direction and the consumer's affine applies it (priors 10).
//
//  THE PAYLOAD is colour: four channels, 0..255 as floats, straight alpha, and alpha 0 where the
//  file says nodata (every colour channel equal to GDAL's nodata value) -- absence, not black.
//  A file of one channel of 16-bit or float is kind HEIGHT, which no stack takes from a file yet:
//  refused by name. The identity is in Structure(): FNV-1a 64 of every byte of the file and of its
//  sidecars, and of the declared crs.
// ================================================================================================
#pragma once

#include "core/FieldLoader.h"

#include <memory>
#include <string>

namespace ga {

// The CRS kind this engine evaluates exactly for an EPSG code, and the UTM zone and hemisphere
// read off it; false, and why, for every other code.
bool CrsOfEpsg(int epsg, CrsKind& kind, int& utmZone, bool& south, std::string* why);
// "EPSG:32617" or "32617" -> 32617; 0 when the text is neither.
int ParseEpsg(const std::string& text);
// The factory. `declared` (may be null): epsg > 0 is the entry's crs, for a file that carries
// none; valueUnit "sRGB byte" says kind colour, any other non-empty unit kind height.
std::unique_ptr<FieldLoader> OpenImageFile(const std::string& path, const GeoRef* declared);
// tif, tiff, png, jpg, jpeg -> OpenImageFile.
void RegisterImageLoaders(LoaderRegistry& reg);
// Why the last OpenImageFile on this thread returned null (the factory's signature has no room).
const std::string& ImageLoaderWhy();

}  // namespace ga
