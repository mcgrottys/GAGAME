# Sources

The LICENSE grants everything and requires nothing. It does ask one thing: if you build on
GAGAME, carry this list forward into your own project, and add GAGAME to it. Not for the
credit, though that is a pleasant side effect, but because every entry below is a place a
project like yours can go for good information, and a reader of your project (human or AI
agent) deserves the same trail we followed. Copy the list, keep the format, add your own.

Entries follow MLA 9 (Modern Language Association), which cites datasets, software, standards
and papers in one style. Access dates are when the project last verified the endpoint. The
GAMEPLAN bibliography (section 11) is the working list this was drawn from; where that list
has a reference and this one does not, GAMEPLAN is authoritative.

## This project

McGrotty, Mark. *GAGAME: A Real-Time, Data-Driven Global Ocean Simulator Built on Geometric
Algebra*. 2026, github.com/mcgrottys/GAGAME. Code under the BSD Zero Clause License; the plan
in GAMEPLAN.md, the mathematics in docs/ALGEBRA.md, the data pipeline in docs/HIERARCHY.md.

## Data: water

National Oceanic and Atmospheric Administration, Center for Operational Oceanographic
Products and Services. *CO-OPS Data API and Metadata API: Water Levels, Harmonic Constituents,
Current Predictions*. NOAA, api.tidesandcurrents.noaa.gov. Accessed 28 Aug. 2026.

---. *Gulf of Maine Operational Forecast System (GoMOFS)*. NOAA, tidesandcurrents.noaa.gov/ofs/gomofs;
mirrored on AWS as noaa-nos-ofs-pds, registry.opendata.aws/noaa-ofs. Accessed 28 Aug. 2026.

---. *CO-OPS THREDDS Data Server (OPeNDAP)*. NOAA, opendap.co-ops.nos.noaa.gov. Accessed 28 Aug. 2026.

National Oceanic and Atmospheric Administration, National Data Buoy Center. *Realtime Buoy
Observations and Directional Wave Spectra (realtime2)*. NOAA, www.ndbc.noaa.gov/data/realtime2.
Accessed 28 Aug. 2026.

National Oceanic and Atmospheric Administration, National Centers for Environmental
Prediction. *Global Forecast System (GFS) and GFS-Wave, 0.25 Degree*. NOAA Open Data
Dissemination on AWS, noaa-gfs-bdp-pds, registry.opendata.aws/noaa-gfs-bdp-pds; subsets via
NOMADS, nomads.ncep.noaa.gov. Accessed 28 Aug. 2026.

---. *High-Resolution Rapid Refresh (HRRR), 3 km*. NOAA Open Data Dissemination on AWS,
noaa-hrrr-bdp-pds and hrrrzarr. Accessed 28 Aug. 2026.

---. *Real-Time Ocean Forecast System (RTOFS), 1/12 Degree*. NOAA Open Data Dissemination on
AWS, noaa-nws-rtofs-pds. Accessed 28 Aug. 2026.

National Oceanic and Atmospheric Administration, CoastWatch. *MUR SST (jplMURSST41) via
ERDDAP*. NOAA, coastwatch.noaa.gov/erddap. Accessed 28 Aug. 2026.

United States Geological Survey. *National Water Information System: Instantaneous Values
Web Service*. USGS, waterservices.usgs.gov/nwis/iv; successor api.waterdata.usgs.gov/ogcapi.
Accessed 28 Aug. 2026.

Hart-Davis, Michael G., et al. *EOT20: A Global Empirical Ocean Tide Model from Multi-Mission
Satellite Altimetry*. SEANOE, 2021, doi:10.17882/79489. Licensed CC BY 4.0.

School for Marine Science and Technology, University of Massachusetts Dartmouth. *Northeast
Coastal Ocean Forecast System (NECOFS), FVCOM GOM3 and MASSBAY*. fvcom.smast.umassd.edu/necofs.
Accessed 28 Aug. 2026.

## Data: bathymetry, elevation, shoreline

GEBCO Compilation Group. *GEBCO 2026 Grid*. General Bathymetric Chart of the Oceans, 2026,
www.gebco.net.

National Oceanic and Atmospheric Administration, National Centers for Environmental
Information. *ETOPO 2022 Global Relief Model*. NOAA NCEI, 2022, doi:10.25921/fd45-gt74.

---. *Continuously Updated Digital Elevation Model (CUDEM), 1/9 Arc-Second Topobathy*. NOAA
Office for Coastal Management, coast.noaa.gov/htdata/raster2/elevation. Accessed 28 Aug. 2026.

National Oceanic and Atmospheric Administration, Office for Coastal Management. *NOAA Coastal
Lidar and DEMs*. NOAA Open Data Dissemination on AWS, noaa-nos-coastal-lidar-pds,
registry.opendata.aws/noaa-coastal-lidar. Accessed Sept. 2026.

National Oceanic and Atmospheric Administration, Office of Coast Survey. *National Bathymetric
Source*. NOAA Open Data Dissemination on AWS, noaa-ocs-nationalbathymetry-pds. Accessed Sept. 2026.

---. *Electronic Navigational Charts (ENC) Direct to GIS*. NOAA, encdirect.noaa.gov. Accessed Oct. 2026.

National Oceanic and Atmospheric Administration, National Geodetic Survey. *Continually
Updated Shoreline Product*. NOAA, geodesy.noaa.gov/dist_shoreline. Accessed Sept. 2026.

Wessel, Paul, and Walter H. F. Smith. *A Global Self-Consistent, Hierarchical,
High-Resolution Geography Database (GSHHG)*, version 2.3.7. 2017,
www.soest.hawaii.edu/pwessel/gshhg; mirrored by NOAA NCEI. Licensed LGPL.

United States Geological Survey. *3D Elevation Program (3DEP) Staged Products*. The National
Map, prd-tnm.s3.amazonaws.com/StagedProducts. Accessed Sept. 2026.

United States Geological Survey, Astrogeology Science Center. *Mars MGS MOLA Global Elevation
Mosaic*. planetarymaps.usgs.gov; source data at pds-geosciences.wustl.edu/mgs. Accessed Aug. 2026.

## Data: imagery, buildings, roads, bridges

Google. *Map Tiles API*. Google Maps Platform, developers.google.com/maps/documentation/tile.
Imagery used under the Google Maps Platform Terms of Service; requires the user's own key.

OpenStreetMap contributors. *OpenStreetMap Planet Data*. OpenStreetMap Foundation,
planet.openstreetmap.org. Licensed ODbL 1.0. Read with *Osmium*, osmcode.org/osmium-tool.

Overture Maps Foundation. *Overture Maps Buildings and Transportation Themes*. STAC catalog,
stac.overturemaps.org. Accessed Oct. 2026.

Federal Highway Administration. *National Bridge Inventory (NBI)*. United States Department
of Transportation, www.fhwa.dot.gov/bridge/nbi.cfm. Accessed Oct. 2026.

United States Department of Agriculture, Farm Service Agency. *National Agriculture Imagery
Program (NAIP)*. Registry of Open Data on AWS, registry.opendata.aws/naip; Microsoft Planetary
Computer, planetarycomputer.microsoft.com/dataset/naip.

## Ocean rendering and simulation

Tessendorf, Jerry. *Simulating Ocean Water*. SIGGRAPH course notes, 2001, updated 2004,
people.computing.clemson.edu/~jtessen/reports/papers_files/coursenotes2004.pdf.

---. *Interactive Water Surfaces*. Game Programming Gems 4, 2004,
people.computing.clemson.edu/~jtessen/reports/papers_files/Interactive_Water_Surfaces.pdf.

Horvath, Christopher J. "Empirical Directional Wave Spectra for Computer Graphics."
*Proceedings of DigiPro 2015*, ACM, 2015. Reference implementation: github.com/blackencino/EncinoWaves.

Bruneton, Eric, Fabrice Neyret, and Nicolas Holzschuch. "Real-Time Realistic Ocean Lighting
Using Seamless Transitions from Geometry to BRDF." *Computer Graphics Forum*, vol. 29, no. 2,
2010, maverick.inria.fr/Publications/2010/BNH10.

Dupuy, Jonathan, and Eric Bruneton. "Real-Time Animation and Rendering of Ocean Whitecaps."
*SIGGRAPH Asia 2012 Technical Briefs*, ACM, 2012. Code: github.com/jdupuy/whitecaps.

Jeschke, Stefan, and Chris Wojtan. "Water Wave Packets." *ACM Transactions on Graphics*,
vol. 36, no. 4, 2017, visualcomputing.ist.ac.at/publications/2017/WWP.

Jeschke, Stefan, et al. "Water Surface Wavelets." *ACM Transactions on Graphics*, vol. 37,
no. 4, 2018, visualcomputing.ist.ac.at/publications/2018/WSW.

Schreck, Camille, Christian Hafner, and Chris Wojtan. "Fundamental Solutions for Water Wave
Animation." *ACM Transactions on Graphics*, vol. 38, no. 4, 2019,
visualcomputing.ist.ac.at/publications/2019/FundamentalSolutionForWaterWave.

Jeschke, Stefan, and Chris Wojtan. "Generalizing Shallow Water Simulations with Dispersive
Surface Waves." *ACM Transactions on Graphics*, vol. 42, no. 4, 2023, doi:10.1145/3592098.

Chentanez, Nuttapong, and Matthias Müller. "Real-Time Simulation of Large Bodies of Water
with Small Scale Details." *Symposium on Computer Animation*, Eurographics, 2010,
matthias-research.github.io/pages/publications/hfFluid.pdf.

Mei, Xing, Philippe Decaudin, and Bao-Gang Hu. "Fast Hydraulic Erosion Simulation and
Visualization on GPU." *Pacific Graphics 2007*, IEEE, 2007.

Dean, Robert G., and Robert A. Dalrymple. *Water Wave Mechanics for Engineers and
Scientists*. World Scientific, 1991.

Havelock, T. H. "Forced Surface-Waves on Water." *Philosophical Magazine*, series 7, vol. 8,
no. 51, 1929, pp. 569-576.

Cox, Charles, and Walter Munk. "Measurement of the Roughness of the Sea Surface from
Photographs of the Sun's Glitter." *Journal of the Optical Society of America*, vol. 44,
no. 11, 1954, pp. 838-850.

Johanson, Claes. *Real-Time Water Rendering: Introducing the Projected Grid Concept*. Master's
thesis, Lund University, 2004.

Losasso, Frank, and Hugues Hoppe. "Geometry Clipmaps: Terrain Rendering Using Nested Regular
Grids." *ACM Transactions on Graphics*, vol. 23, no. 3, 2004, hhoppe.com/proj/geomclipmap.

Strugar, Filip. *Continuous Distance-Dependent Level of Detail for Rendering Heightmaps
(CDLOD)*. 2009, github.com/fstrugar/CDLOD.

Wave Harmonic. *Crest Ocean System*. github.com/wave-harmonic/crest.

Rare Ltd. "The Technical Art of Sea of Thieves." *SIGGRAPH 2018 Talks*, ACM, 2018.

Malan, Hugh. "Water in Horizon Forbidden West." *Advances in Real-Time Rendering in Games*,
SIGGRAPH 2022 course.

Tcheblokov, Tim. "Ocean Simulation and Rendering in War Thunder." *China Game Developers
Conference*, 2015.

Mihelich, Mark, and Tim Tcheblokov. "Wakes, Explosions and Lighting: Interactive Water
Simulation in Atlas." *Game Developers Conference*, 2019.

2Retr0. *GodotOceanWaves*. github.com/2Retr0/GodotOceanWaves. Licensed MIT.

gasgiant. *FFT-Ocean*. github.com/gasgiant/FFT-Ocean. Licensed MIT.

## Geometric algebra

Hestenes, David. *Space-Time Algebra*. Gordon and Breach, 1966; 2nd ed., Birkhäuser, 2015.

---. "Oersted Medal Lecture 2002: Reforming the Mathematical Language of Physics." *American
Journal of Physics*, vol. 71, no. 2, 2003, pp. 104-121.

Doran, Chris, and Anthony Lasenby. *Geometric Algebra for Physicists*. Cambridge UP, 2003.

Dorst, Leo, Daniel Fontijne, and Stephen Mann. *Geometric Algebra for Computer Science: An
Object-Oriented Approach to Geometry*. Morgan Kaufmann, 2007.

Baylis, William E. *Electrodynamics: A Modern Geometric Approach*. Birkhäuser, 1999.

Han, D., Y. S. Kim, and M. E. Noz. "Polarization Optics and Bilinear Representation of the
Lorentz Group." *Journal of the Optical Society of America A*, vol. 14, 1997.

Cibura, Carsten, and Dietmar Hildenbrand. "Geometric Algebra Approach to Fluid Dynamics."
*AGACSE 2008*, 2008, gaalop.de.

Panakkal, Susan Mathew, et al. "A Geometric Algebra-Based Approach for Fluid Dynamics."
*Physics of Fluids*, vol. 32, 2020.

Marmanis, Haralambos. "Analogy between the Navier-Stokes Equations and Maxwell's Equations:
Application to Turbulence." *Physics of Fluids*, vol. 10, no. 6, 1998.

Dressel, Justin, Konstantin Y. Bliokh, and Franco Nori. "Spacetime Algebra as a Powerful
Tool for Electromagnetism." *Physics Reports*, vol. 589, 2015, arxiv.org/abs/1411.5002.

Ebling, Julia, and Gerik Scheuermann. "Clifford Convolution and Pattern Matching on Vector
Fields." *IEEE Visualization 2003*, IEEE, 2003.

---. "Clifford Fourier Transform on Vector Fields." *IEEE Transactions on Visualization and
Computer Graphics*, vol. 11, no. 4, 2005.

Hitzer, Eckhard. *Quaternion and Clifford Fourier Transforms*. CRC Press, 2021.

Ell, Todd A., and Stephen J. Sangwine. "Hypercomplex Fourier Transforms of Color Images."
*IEEE Transactions on Image Processing*, vol. 16, no. 1, 2007.

Pei, Soo-Chang, Jian-Jiun Ding, and Ja-Han Chang. "Efficient Implementation of Quaternion
Fourier Transform, Convolution, and Correlation by 2-D Complex FFT." *IEEE Transactions on
Signal Processing*, vol. 49, no. 11, 2001.

Brandstetter, Johannes, et al. "Clifford Neural Layers for PDE Modeling." *ICLR 2023*, 2023,
github.com/microsoft/cliffordlayers.

Ruhe, David, et al. "Geometric Clifford Algebra Networks." *ICML 2023*, 2023.

Ruhe, David, Johannes Brandstetter, and Patrick Forré. "Clifford Group Equivariant Neural
Networks." *NeurIPS 2023*, 2023.

Zhdanov, Maksim, et al. "Clifford-Steerable Convolutional Neural Networks." *ICML 2024*, 2024.

Brehmer, Johann, et al. "Geometric Algebra Transformer." *NeurIPS 2023*, 2023.

Pepe, Alberto, et al. "Fengbo: A Clifford Neural Operator Pipeline for 3D PDEs in
Computational Fluid Dynamics." *ICLR 2025*, 2025.

Hildenbrand, Dietmar, et al. *GAALOP: Geometric Algebra Algorithms Optimizer*. gaalop.de.

De Keninck, Steven. *Look, Ma, No Matrices!* SIGGRAPH 2024 Talks, ACM, 2024. See also
bivector.net.

Roelfs, Martin, and Steven De Keninck. *kingdon: Geometric Algebra in Python*.
github.com/tBuLi/kingdon; arxiv.org/abs/2503.10451.

Breuils, Stéphane, Vincent Nozick, and Laurent Fuchs. "Garamon: A Geometric Algebra Library
Generator." *Advances in Applied Clifford Algebras*, vol. 29, 2019.

Lengyel, Eric. *Projective Geometric Algebra Illuminated*. Terathon Software, 2024. Code:
Terathon-Math-Library, licensed MIT.

## Sparse GPU residency, virtual texturing, virtual globes

Microsoft. *Direct3D 12 Programming Guide: Tiled Resources, D3D12_TILED_RESOURCES_TIER,
UpdateTileMappings, Sampler Feedback*. Microsoft Learn, learn.microsoft.com/windows/win32/direct3d12.
Accessed 2026.

---. *DirectX Specs: D3D12 Tiled Resource Tier 4*. microsoft.github.io/DirectX-Specs. Accessed 2026.

---. *DirectStorage*. github.com/microsoft/DirectStorage.

---. *DirectX Shader Compiler (DXC)*. github.com/microsoft/DirectXShaderCompiler.

Intel Game Technology Development. *Sampler Feedback Streaming*.
github.com/GameTechDev/SamplerFeedbackStreaming.

Dunn, Alex. "Sparse Fluid Simulation in DirectX." *Game Developers Conference*, NVIDIA, 2015.

Setaluri, Rajsekhar, et al. "SPGrid: A Sparse Paged Grid Structure Applied to Adaptive Smoke
Simulation." *ACM Transactions on Graphics*, vol. 33, no. 6, 2014.

Museth, Ken. "VDB: High-Resolution Sparse Volumes with Dynamic Topology." *ACM Transactions on
Graphics*, vol. 32, no. 3, 2013. See also *NanoVDB*, NVIDIA.

van Waveren, J. M. P. *Software Virtual Textures*. id Software, 2012,
mrelusive.com/publications/papers/Software-Virtual-Textures.pdf.

Obert, Juraj, et al. "Virtual Texturing in Software and Hardware." *SIGGRAPH 2012 Courses*,
ACM, 2012.

Chen, Ka. "Adaptive Virtual Texture Rendering in Far Cry 4." *Game Developers Conference*, 2015.

Shlomnik, Daniel. "How Virtual Textures Really Work." *shlom.dev*,
www.shlom.dev/articles/how-virtual-textures-really-work.

Cozzi, Patrick, and Kevin Ring. *3D Engine Design for Virtual Globes*. CRC Press, 2011.

Segal, Mark, et al. "Fast Shadows and Lighting Effects Using Texture Mapping." *Computer
Graphics (SIGGRAPH '92)*, vol. 26, no. 2, 1992, doi:10.1145/142920.134071.

## Standards and formats

International Organization for Standardization. *ISO/IEC 11172-4: MPEG Audio Compliance*.
Summary at www.underbit.com/resources/mpeg/audio/compliance.

World Meteorological Organization. *FM 92 GRIB Edition 2*. WMO Manual on Codes, No. 306.

Unidata. *NetCDF Classic Format Specification*. UCAR, docs.unidata.ucar.edu/netcdf-c.

The HDF Group. *HDF5 File Format Specification*. www.hdfgroup.org.

Open Geospatial Consortium. *GeoTIFF Standard*, version 1.1. OGC 19-008r4, 2019.

OpenStreetMap contributors. *PBF Format*. OpenStreetMap Wiki, wiki.openstreetmap.org/wiki/PBF_Format.
