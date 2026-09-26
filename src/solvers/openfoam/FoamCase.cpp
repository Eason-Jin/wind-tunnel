#include "solvers/openfoam/FoamCase.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;

namespace solvers::foam {

namespace {

std::string num(double v)
{
    char buf[48];
    std::snprintf(buf, sizeof buf, "%.9g", v);
    return buf;
}

std::string vec(const glm::dvec3& v) { return "(" + num(v.x) + " " + num(v.y) + " " + num(v.z) + ")"; }

void writeFoamFile(const fs::path& path, const std::string& cls, const std::string& body)
{
    fs::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::trunc);
    if (!out)
        throw std::runtime_error("OpenFOAM: cannot write " + path.string());
    out << "FoamFile\n{\n    version     2.0;\n    format      ascii;\n    class       " << cls
        << ";\n    object      " << path.filename().string() << ";\n}\n\n"
        << body << "\n// Written by windtunnel\n";
    if (!out)
        throw std::runtime_error("OpenFOAM: failed writing " + path.string());
}

// A single hex block from lo to hi with the given cells and six boundary faces.
// Vertex order: 0..3 at z=lo, 4..7 at z=hi, counter-clockwise from (lo.x, lo.y).
std::string blockMeshDict(const glm::dvec3& lo, const glm::dvec3& hi, const glm::ivec3& cells, const std::string& boundary)
{
    std::ostringstream s;
    s << "scale 1;\n\nvertices\n(\n";
    const glm::dvec3 v[8] = {{lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, hi.y, lo.z}, {lo.x, hi.y, lo.z},
                             {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}};
    for (const auto& p : v)
        s << "    " << vec(p) << "\n";
    s << ");\n\nblocks\n(\n    hex (0 1 2 3 4 5 6 7) (" << cells.x << " " << cells.y << " " << cells.z
      << ") simpleGrading (1 1 1)\n);\n\nedges\n(\n);\n\nboundary\n(\n"
      << boundary << ");\n\nmergePatchPairs\n(\n);\n";
    return s.str();
}

// Face lists of the hex above (outward-pointing).
constexpr const char* kFaceXMin = "(0 4 7 3)";
constexpr const char* kFaceXMax = "(2 6 5 1)";
constexpr const char* kFaceYMin = "(1 5 4 0)";
constexpr const char* kFaceYMax = "(3 7 6 2)";
constexpr const char* kFaceZMin = "(0 3 2 1)";
constexpr const char* kFaceZMax = "(4 5 6 7)";

std::string controlDict(const std::string& app, int endTime, int writeInterval, const char* format = "ascii",
                        const std::string& functions = "")
{
    return "application     " + app +
           ";\nstartFrom       startTime;\nstartTime       0;\nstopAt          endTime;\nendTime         " +
           std::to_string(endTime) + ";\ndeltaT          1;\nwriteControl    timeStep;\nwriteInterval   " +
           std::to_string(writeInterval) + ";\npurgeWrite      0;\nwriteFormat     " + format +
           ";\nwritePrecision  7;\nwriteCompression off;\n"
           "timeFormat      general;\ntimePrecision   6;\nrunTimeModifiable false;\n" + functions;
}

// Drag monitoring, an automatic early stop and wake averaging.
// - The run stops only when the drag has settled AND the pressure residual is
//   low: an unsteady wake makes the drag oscillate, and its current value
//   would otherwise meet the average by chance.
// - U and p are averaged from `minIterations` on. A bluff body's wake never
//   settles in a steady solver, so the last iteration is just one snapshot of
//   it; the mean is what the result shows (see OpenFoamSolver).
std::string convergenceFunctions(const CaseSpec& s, int minIterations)
{
    const glm::dvec3 c = (glm::dvec3(s.body.min) + glm::dvec3(s.body.max)) * 0.5;
    const glm::dvec3 size = glm::dvec3(s.body.max) - glm::dvec3(s.body.min);
    const double lRef = std::max(size.x, 1e-6);
    const double aRef = std::max(size.y * size.z, 1e-12); // frontal box area: only the scale of Cd depends on it
    std::ostringstream f;
    f << "\nfunctions\n{\n"
      << "    forceCoeffs\n    {\n"
      << "        type            forceCoeffs;\n        libs            (forces);\n"
      << "        writeControl    timeStep;\n        writeInterval   1;\n        log             false;\n"
      << "        patches         (\"body.*\");\n        p               p;\n        U               U;\n"
      << "        rho             rhoInf;\n        rhoInf          1;\n"
      << "        liftDir         (0 0 1);\n        dragDir         (1 0 0);\n        pitchAxis       (0 1 0);\n"
      << "        CofR            (" << c.x << " " << c.y << " " << c.z << ");\n"
      << "        magUInf         " << s.inletSpeed << ";\n        lRef            " << lRef
      << ";\n        Aref            " << aRef << ";\n    }\n"
      << "    average\n    {\n"
      << "        type            fieldAverage;\n        libs            (fieldFunctionObjects);\n"
      << "        timeStart       " << minIterations << ";\n        writeControl    writeTime;\n"
      << "        fields\n        (\n"
      << "            U { mean on; prime2Mean off; base iteration; }\n"
      << "            p { mean on; prime2Mean off; base iteration; }\n"
      << "        );\n    }\n"
      << "    converged\n    {\n"
      << "        type            runTimeControl;\n        libs            (utilityFunctionObjects);\n"
      << "        timeStart       " << minIterations << ";\n"
      << "        conditions\n        {\n            drag\n            {\n"
      << "                type            average;\n                functionObject  forceCoeffs;\n"
      << "                fields          (Cd);\n                tolerance       1e-3;\n"
      << "                window          50;\n                windowType      exact;\n"
      << "                groupID         1;\n            }\n"
      << "            residual\n            {\n"
      << "                type            equationInitialResidual;\n                fields          (p);\n"
      << "                value           1e-4;\n                mode            minimum;\n"
      << "                groupID         1;\n            }\n"
      << "        }\n        satisfiedAction end;\n    }\n}\n";
    return f.str();
}

const char* kFvSchemes = R"FOAM(ddtSchemes
{
    default         steadyState;
}

gradSchemes
{
    default         Gauss linear;
    grad(U)         cellLimited Gauss linear 1;
    grad(k)         cellLimited Gauss linear 1;
    grad(omega)     cellLimited Gauss linear 1;
}

divSchemes
{
    default         none;
    div(phi,U)      bounded Gauss linearUpwind grad(U);
    turbulence      bounded Gauss upwind;
    div(phi,k)      $turbulence;
    div(phi,omega)  $turbulence;
    div((nuEff*dev2(T(grad(U))))) Gauss linear;
}

laplacianSchemes
{
    default         Gauss linear limited corrected 0.33;
}

interpolationSchemes
{
    default         linear;
}

snGradSchemes
{
    default         limited corrected 0.33;
}

wallDist
{
    method          meshWave;
}
)FOAM";

const char* kFvSolution = R"FOAM(solvers
{
    p
    {
        solver          GAMG;
        smoother        GaussSeidel;
        tolerance       1e-7;
        relTol          0.01;
    }

    Phi
    {
        $p;
    }

    "(U|k|omega)"
    {
        solver          smoothSolver;
        smoother        GaussSeidel;
        tolerance       1e-8;
        relTol          0.1;
        nSweeps         1;
    }
}

SIMPLE
{
    nNonOrthogonalCorrectors 0;
    consistent      yes;
    residualControl
    {
        p               1e-4;
        U               1e-5;
        "(k|omega)"     1e-5;
    }
}

potentialFlow
{
    nNonOrthogonalCorrectors 10;
}

relaxationFactors
{
    equations
    {
        U               0.9;
        k               0.7;
        omega           0.7;
    }
}

cache
{
    grad(U);
}
)FOAM";

std::string snappyDict(const CaseSpec& s)
{
    const std::string S = std::to_string(s.surfaceLevel);
    const std::string R = std::to_string(s.regionLevel);
    std::ostringstream d;
    d << "castellatedMesh true;\nsnap            true;\naddLayers       false;\n\n"
      << "geometry\n{\n    body.stl\n    {\n        type triSurfaceMesh;\n        name body;\n    }\n"
      << "    refinementBox\n    {\n        type searchableBox;\n        min  " << vec(s.refineMin) << ";\n        max  "
      << vec(s.refineMax) << ";\n    }\n}\n\n"
      << "castellatedMeshControls\n{\n"
      << "    maxLocalCells       2000000;\n    maxGlobalCells      20000000;\n    minRefinementCells  10;\n"
      << "    maxLoadUnbalance    0.10;\n    nCellsBetweenLevels 3;\n"
      << "    features\n    (\n        {\n            file \"body.eMesh\";\n            level " << S << ";\n        }\n    );\n"
      << "    refinementSurfaces\n    {\n        body\n        {\n            level (" << S << " " << S
      << ");\n            patchInfo\n            {\n                type wall;\n            }\n        }\n    }\n"
      << "    resolveFeatureAngle 30;\n"
      << "    refinementRegions\n    {\n        refinementBox\n        {\n            mode inside;\n            levels ((1e15 " << R
      << "));\n        }\n    }\n"
      << "    locationInMesh " << vec(s.locationInMesh) << ";\n    allowFreeStandingZoneFaces true;\n}\n\n"
      << R"FOAM(snapControls
{
    nSmoothPatch            3;
    tolerance               2.0;
    nSolveIter              30;
    nRelaxIter              5;
    nFeatureSnapIter        10;
    implicitFeatureSnap     false;
    explicitFeatureSnap     true;
    multiRegionFeatureSnap  false;
}

addLayersControls
{
    relativeSizes           true;
    layers
    {
    }
    expansionRatio          1.0;
    finalLayerThickness     0.3;
    minThickness            0.1;
    nGrow                   0;
    featureAngle            60;
    slipFeatureAngle        30;
    nRelaxIter              3;
    nSmoothSurfaceNormals   1;
    nSmoothNormals          3;
    nSmoothThickness        10;
    maxFaceThicknessRatio   0.5;
    maxThicknessToMedialRatio 0.3;
    minMedialAxisAngle      90;
    nBufferCellsNoExtrude   0;
    nLayerIter              50;
}

meshQualityControls
{
    #include "meshQualityDict"
    nSmoothScale    4;
    errorReduction  0.75;
}

mergeTolerance 1e-6;
)FOAM";
    return d.str();
}

// Boundary-field entries shared by the turbulence scalars k and omega.
std::string turbulenceScalar(const CaseSpec& s, const std::string& value, const std::string& wallFunction)
{
    const std::string v = "uniform " + value;
    std::string floor = s.groundPlane ? "        type            " + wallFunction + ";\n        value           " + v + ";\n"
                                      : "        type            slip;\n";
    return "internalField   " + v +
           ";\n\nboundaryField\n{\n    #includeEtc \"caseDicts/setConstraintTypes\"\n\n"
           "    inlet\n    {\n        type            fixedValue;\n        value           " +
           v + ";\n    }\n    outlet\n    {\n        type            inletOutlet;\n        inletValue      " + v +
           ";\n        value           " + v + ";\n    }\n    sides\n    {\n        type            slip;\n    }\n    floor\n    {\n" +
           floor + "    }\n    \"body.*\"\n    {\n        type            " + wallFunction + ";\n        value           " + v +
           ";\n    }\n}\n";
}

} // namespace

CaseSpec makeCaseSpec(const CaseInputs& in, const core::TunnelDomain& domain)
{
    CaseSpec s;
    s.body = in.bodyBounds;
    s.domain = domain;
    s.groundPlane = in.groundPlane;
    s.inletSpeed = in.inletSpeed;
    s.nu = in.nu;
    s.iterations = std::max(1, in.iterations);
    s.processors = std::max(1, in.processors);

    const glm::dvec3 bmin(s.body.min), bmax(s.body.max);
    const glm::dvec3 size = bmax - bmin;
    const double L = std::max({size.x, size.y, size.z, 1e-6});

    // Background cells ~L/8; the body gets `refinementLevel` halvings on top.
    const double h0 = L / 8.0;
    s.surfaceLevel = std::clamp(in.refinementLevel, 0, 8);
    s.regionLevel = std::max(0, s.surfaceLevel - 1);
    const double hSurface = h0 / std::pow(2.0, s.surfaceLevel);

    s.meshMin = glm::dvec3(domain.box.min);
    s.meshMax = glm::dvec3(domain.box.max);
    if (s.groundPlane) {
        // Raise the CFD floor slightly into the body: the body then cuts the
        // floor along a finite contact patch (like a tyre footprint) instead
        // of touching it at a point/line, which snappyHexMesh handles badly.
        // Kept below a quarter output cell so every output cell centre is
        // still inside the CFD mesh.
        s.meshMin.z += std::min(0.5 * hSurface, 0.25 * static_cast<double>(domain.cellSize));
    }

    const glm::dvec3 extent = s.meshMax - s.meshMin;
    s.backgroundCells = glm::ivec3(std::max(1, static_cast<int>(std::lround(extent.x / h0))),
                                   std::max(1, static_cast<int>(std::lround(extent.y / h0))),
                                   std::max(1, static_cast<int>(std::lround(extent.z / h0))));

    // Near-body + wake refinement box, clipped to the domain.
    const double m = 0.25 * L;
    s.refineMin = {bmin.x - m, bmin.y - m, s.groundPlane ? s.meshMin.z - m : bmin.z - m};
    s.refineMax = {bmax.x + 1.5 * L, bmax.y + m, bmax.z + m};
    s.refineMin = glm::max(s.refineMin, s.meshMin - glm::dvec3(m));
    s.refineMax = glm::min(s.refineMax, s.meshMax + glm::dvec3(m));

    // A fluid point halfway between the inlet/side walls and the body,
    // nudged off any cell face.
    const glm::dvec3 nudge = h0 * glm::dvec3(0.01371, 0.00713, 0.01129);
    s.locationInMesh = {s.meshMin.x + 0.5 * (bmin.x - s.meshMin.x), s.meshMin.y + 0.5 * (bmin.y - s.meshMin.y),
                        s.meshMin.z + 0.5 * extent.z};
    s.locationInMesh += nudge;
    if (bmin.y - s.meshMin.y <= 2.0 * h0 && bmin.x - s.meshMin.x > 2.0 * h0)
        s.locationInMesh.y = 0.5 * (bmin.y + bmax.y) + nudge.y; // no side gap: go upstream instead

    // Inlet turbulence: intensity 1 %, mixing length 0.1 L.
    const double intensity = 0.01;
    const double lengthScale = 0.1 * L;
    s.k = std::max(1.5 * std::pow(in.inletSpeed * intensity, 2.0), 1e-10);
    s.omega = std::sqrt(s.k) / (std::pow(0.09, 0.25) * lengthScale);
    return s;
}

void writeMainCase(const fs::path& c, const CaseSpec& s)
{
    const fs::path sys = c / "system", cst = c / "constant", zero = c / "0.orig";

    // Binary I/O for the (large) CFD case; the small output grid case stays
    // ASCII because our reader parses it.
    writeFoamFile(sys / "controlDict", "dictionary",
                  controlDict("simpleFoam", s.iterations, s.iterations, "binary",
                              convergenceFunctions(s, std::min(s.iterations, std::max(100, s.iterations / 4)))));
    writeFoamFile(sys / "fvSchemes", "dictionary", kFvSchemes);
    writeFoamFile(sys / "fvSolution", "dictionary", kFvSolution);
    writeFoamFile(sys / "meshQualityDict", "dictionary", "#includeEtc \"caseDicts/meshQualityDict\"\n\nminFaceWeight 0.02;\n");
    writeFoamFile(sys / "decomposeParDict", "dictionary",
                  "numberOfSubdomains " + std::to_string(s.processors) + ";\n\nmethod          scotch;\n");
    writeFoamFile(sys / "surfaceFeatureExtractDict", "dictionary", R"FOAM(body.stl
{
    extractionMethod    extractFromSurface;
    includedAngle       150;
    subsetFeatures
    {
        nonManifoldEdges    no;
        openEdges           yes;
    }
    writeObj            no;
}
)FOAM");
    const std::string floorType = s.groundPlane ? "wall" : "patch";
    const std::string boundary = std::string("    inlet\n    {\n        type patch;\n        faces (") + kFaceXMin +
                                 ");\n    }\n    outlet\n    {\n        type patch;\n        faces (" + kFaceXMax +
                                 ");\n    }\n    floor\n    {\n        type " + floorType + ";\n        faces (" + kFaceZMin +
                                 ");\n    }\n    sides\n    {\n        type patch;\n        faces (" + kFaceYMin + " " +
                                 kFaceYMax + " " + kFaceZMax + ");\n    }\n";
    writeFoamFile(sys / "blockMeshDict", "dictionary", blockMeshDict(s.meshMin, s.meshMax, s.backgroundCells, boundary));
    writeFoamFile(sys / "snappyHexMeshDict", "dictionary", snappyDict(s));

    writeFoamFile(cst / "transportProperties", "dictionary", "transportModel  Newtonian;\n\nnu              " + num(s.nu) + ";\n");
    writeFoamFile(cst / "turbulenceProperties", "dictionary",
                  "simulationType  RAS;\n\nRAS\n{\n    RASModel        kOmegaSST;\n    turbulence      on;\n    printCoeffs     on;\n}\n");

    const std::string Uin = "uniform (" + num(s.inletSpeed) + " 0 0)";
    const std::string floorU = s.groundPlane
                                   ? "        type            fixedValue;   // moving ground at the free-stream speed\n        value           " + Uin + ";\n"
                                   : "        type            slip;\n";
    writeFoamFile(zero / "U", "volVectorField",
                  "dimensions      [0 1 -1 0 0 0 0];\n\ninternalField   " + Uin +
                      ";\n\nboundaryField\n{\n    #includeEtc \"caseDicts/setConstraintTypes\"\n\n"
                      "    inlet\n    {\n        type            fixedValue;\n        value           " + Uin +
                      ";\n    }\n    outlet\n    {\n        type            inletOutlet;\n        inletValue      uniform (0 0 0);\n        value           " +
                      Uin + ";\n    }\n    sides\n    {\n        type            slip;\n    }\n    floor\n    {\n" + floorU +
                      "    }\n    \"body.*\"\n    {\n        type            noSlip;\n    }\n}\n");
    writeFoamFile(zero / "p", "volScalarField",
                  "dimensions      [0 2 -2 0 0 0 0];\n\ninternalField   uniform 0;\n\nboundaryField\n{\n"
                  "    #includeEtc \"caseDicts/setConstraintTypes\"\n\n"
                  "    inlet\n    {\n        type            zeroGradient;\n    }\n"
                  "    outlet\n    {\n        type            fixedValue;\n        value           uniform 0;\n    }\n"
                  "    sides\n    {\n        type            slip;\n    }\n"
                  "    floor\n    {\n        type            zeroGradient;\n    }\n"
                  "    \"body.*\"\n    {\n        type            zeroGradient;\n    }\n}\n");
    writeFoamFile(zero / "k", "volScalarField", "dimensions      [0 2 -2 0 0 0 0];\n\n" + turbulenceScalar(s, num(s.k), "kqRWallFunction"));
    writeFoamFile(zero / "omega", "volScalarField",
                  "dimensions      [0 0 -1 0 0 0 0];\n\n" + turbulenceScalar(s, num(s.omega), "omegaWallFunction"));
    const std::string floorNut = s.groundPlane ? "nutkWallFunction" : "calculated";
    writeFoamFile(zero / "nut", "volScalarField",
                  "dimensions      [0 2 -1 0 0 0 0];\n\ninternalField   uniform 0;\n\nboundaryField\n{\n"
                  "    #includeEtc \"caseDicts/setConstraintTypes\"\n\n"
                  "    \"(inlet|outlet|sides)\"\n    {\n        type            calculated;\n        value           uniform 0;\n    }\n"
                  "    floor\n    {\n        type            " + floorNut + ";\n        value           uniform 0;\n    }\n"
                  "    \"body.*\"\n    {\n        type            nutkWallFunction;\n        value           uniform 0;\n    }\n}\n");
}

void writeGridCase(const fs::path& g, const core::TunnelDomain& domain)
{
    const fs::path sys = g / "system";
    writeFoamFile(sys / "controlDict", "dictionary", controlDict("mapFields", 1, 1));
    writeFoamFile(sys / "fvSchemes", "dictionary",
                  "ddtSchemes { default steadyState; }\ngradSchemes { default Gauss linear; }\ndivSchemes { default none; }\n"
                  "laplacianSchemes { default Gauss linear corrected; }\ninterpolationSchemes { default linear; }\n"
                  "snGradSchemes { default corrected; }\n");
    writeFoamFile(sys / "fvSolution", "dictionary", "solvers\n{\n}\n");
    writeFoamFile(sys / "mapFieldsDict", "dictionary", "patchMap        ( );\n\ncuttingPatches  ( );\n");
    const std::string boundary = std::string("    walls\n    {\n        type patch;\n        faces (") + kFaceXMin + " " +
                                 kFaceXMax + " " + kFaceYMin + " " + kFaceYMax + " " + kFaceZMin + " " + kFaceZMax + ");\n    }\n";
    writeFoamFile(sys / "blockMeshDict", "dictionary",
                  blockMeshDict(glm::dvec3(domain.box.min), glm::dvec3(domain.box.max), domain.cells, boundary));

    const std::string sentinel = num(kUnmappedSentinel);
    writeFoamFile(g / "0" / "U", "volVectorField",
                  "dimensions      [0 1 -1 0 0 0 0];\n\ninternalField   uniform (" + sentinel +
                      " 0 0);\n\nboundaryField\n{\n    walls\n    {\n        type            zeroGradient;\n    }\n}\n");
    writeFoamFile(g / "0" / "p", "volScalarField",
                  "dimensions      [0 2 -2 0 0 0 0];\n\ninternalField   uniform 0;\n\nboundaryField\n{\n"
                  "    walls\n    {\n        type            zeroGradient;\n    }\n}\n");
}

} // namespace solvers::foam
