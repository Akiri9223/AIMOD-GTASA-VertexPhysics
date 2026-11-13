#include "GeometryCloneTest.h"
#include "GeometryPhysics.h"
#include "RenderWare.h"
#include "rpskin.h"
#include "rphanim.h"

#include <windows.h>
#include <sstream>
#include <string>
#include <cstring>
#include <vector>
#include <cstdint>

// ============================================================
// Pointer validation
// ============================================================

static bool CheckPointer(
    const void* pointer,
    const char* name,
    std::ostringstream& log)
{
    if (!pointer)
    {
        log << "[CloneTest] ERROR: " << name
            << " is null.\n";
        return false;
    }

    return true;
}

static bool CheckIndependentPointer(
    const void* source,
    const void* clone,
    const char* name,
    std::ostringstream& log)
{
    const bool independent =
        source != nullptr &&
        clone != nullptr &&
        source != clone;

    log << "[CloneTest] " << name
        << " independent=" << independent
        << " source=" << source
        << " clone=" << clone
        << "\n";

    return independent;
}

// ============================================================
// Mesh diagnostics
// ============================================================

static void DumpGeometryMeshDiag(
    const char* stage,
    RpGeometry* geometry,
    std::ostringstream& log)
{
    std::ostringstream block;

    block << "\n";
    block << "========== [MeshDiag] "
        << stage << " ==========\n";

    if (!geometry)
    {
        block << "[MeshDiag] geometry=NULL\n";
        block << "========================================\n";

        log << block.str();

        const std::string message = block.str();
        OutputDebugStringA(message.c_str());
        return;
    }

    block << "[MeshDiag] geometry=" << geometry << "\n";
    block << "[MeshDiag] refCount=" << geometry->refCount << "\n";
    block << "[MeshDiag] mesh=" << geometry->mesh << "\n";
    block << "[MeshDiag] repEntry=" << geometry->repEntry << "\n";

    if (geometry->mesh)
    {
        RpMeshHeader* mesh = geometry->mesh;

        block << "[MeshDiag] mesh flags=0x"
            << std::hex << mesh->flags << std::dec << "\n";
        block << "[MeshDiag] mesh numMeshes=" << mesh->numMeshes << "\n";
        block << "[MeshDiag] mesh totalIndices=" << mesh->totalIndicesInMesh << "\n";
        block << "[MeshDiag] mesh firstMeshOffset=" << mesh->firstMeshOffset << "\n";
    }
    else
    {
        block << "[MeshDiag] mesh header is NULL.\n";
    }

    block << "========================================\n";

    log << block.str();

    const std::string message = block.str();
    OutputDebugStringA(message.c_str());
}

// ============================================================
// Triangle diagnostics
// ============================================================

static void DumpTriangleInfo(
    const char* label,
    RpGeometry* geometry,
    std::ostringstream& log)
{
    std::ostringstream block;

    block << "\n========== [TriangleDiag] "
        << label << " ==========\n";

    if (!geometry)
    {
        block << "[TriangleDiag] geometry=NULL\n";
        log << block.str();
        const std::string message = block.str();
        OutputDebugStringA(message.c_str());
        return;
    }

    const RwInt32 vertexCount = geometry->numVertices;
    const RwInt32 triangleCount = geometry->numTriangles;
    const RwInt32 materialCount = geometry->matList.numMaterials;

    block << "[TriangleDiag] geometry=" << geometry << "\n";
    block << "[TriangleDiag] vertices=" << vertexCount
        << " triangles=" << triangleCount
        << " materials=" << materialCount << "\n";

    block << "[TriangleDiag] triangleArray=" << geometry->triangles
        << " materialArray=" << geometry->matList.materials << "\n";

    if (vertexCount < 0 ||
        triangleCount < 0 ||
        materialCount < 0)
    {
        block << "[TriangleDiag] ERROR: Invalid geometry dimensions.\n";
        log << block.str();
        const std::string message = block.str();
        OutputDebugStringA(message.c_str());
        return;
    }

    if (triangleCount > 0 && !geometry->triangles)
    {
        block << "[TriangleDiag] ERROR: triangles=NULL\n";
        log << block.str();
        const std::string message = block.str();
        OutputDebugStringA(message.c_str());
        return;
    }

    if (materialCount > 0 && !geometry->matList.materials)
    {
        block << "[TriangleDiag] ERROR: material array=NULL\n";
        log << block.str();
        const std::string message = block.str();
        OutputDebugStringA(message.c_str());
        return;
    }

    RwInt32 invalidVertexIndices = 0;
    RwInt32 invalidMaterialIndices = 0;
    RwInt32 zeroMaterialIndices = 0;
    RwInt32 nonZeroMaterialIndices = 0;
    RwUInt16 maxMaterialIndex = 0;

    for (RwInt32 i = 0; i < triangleCount; ++i)
    {
        const RpTriangle& triangle = geometry->triangles[i];

        for (int j = 0; j < 3; ++j)
        {
            if (triangle.vertIndex[j] >= vertexCount)
                ++invalidVertexIndices;
        }

        if (triangle.matIndex >= materialCount)
            ++invalidMaterialIndices;

        if (triangle.matIndex == 0)
            ++zeroMaterialIndices;
        else
            ++nonZeroMaterialIndices;

        if (triangle.matIndex > maxMaterialIndex)
            maxMaterialIndex = triangle.matIndex;
    }

    block << "[TriangleDiag] invalidVertexIndices=" << invalidVertexIndices << "\n";
    block << "[TriangleDiag] invalidMaterialIndices=" << invalidMaterialIndices << "\n";
    block << "[TriangleDiag] zeroMaterialIndices=" << zeroMaterialIndices << "\n";
    block << "[TriangleDiag] nonZeroMaterialIndices=" << nonZeroMaterialIndices << "\n";
    block << "[TriangleDiag] maxMaterialIndex=" << maxMaterialIndex << "\n";

    const RwInt32 sampleCount = triangleCount < 8 ? triangleCount : 8;

    for (RwInt32 i = 0; i < sampleCount; ++i)
    {
        const RpTriangle& triangle = geometry->triangles[i];

        block << "[TriangleDiag] triangle[" << i << "]"
            << " vertices=("
            << triangle.vertIndex[0] << ","
            << triangle.vertIndex[1] << ","
            << triangle.vertIndex[2] << ")"
            << " matIndex=" << triangle.matIndex << "\n";
    }

    block << "========================================\n";

    log << block.str();

    const std::string message = block.str();
    OutputDebugStringA(message.c_str());
}

// ============================================================
// Copy MaterialList
// ============================================================

static bool CopyGeometryMaterialList(
    RpGeometry* source,
    RpGeometry* clone,
    std::ostringstream& log)
{
    OutputDebugStringA("[CloneTest] STEP D: Material list copy entered.\n");

    if (!source || !clone)
    {
        log << "[CloneTest] ERROR: MaterialList source or clone is null.\n";
        return false;
    }

    const RwInt32 sourceMaterialCount = source->matList.numMaterials;

    log << "[CloneTest] source material count=" << sourceMaterialCount << "\n";
    log << "[CloneTest] source material array=" << source->matList.materials << "\n";
    log << "[CloneTest] clone material count before=" << clone->matList.numMaterials << "\n";
    log << "[CloneTest] clone material array before=" << clone->matList.materials << "\n";

    if (sourceMaterialCount < 0 ||
        (sourceMaterialCount > 0 && !source->matList.materials))
    {
        log << "[CloneTest] ERROR: Invalid source material list.\n";
        return false;
    }

    OutputDebugStringA("[CloneTest] BEFORE _rpMaterialListCopy.\n");

    RpMaterialList* copiedList =
        _rpMaterialListCopy(&clone->matList, &source->matList);

    OutputDebugStringA("[CloneTest] AFTER _rpMaterialListCopy.\n");

    if (!copiedList)
    {
        log << "[CloneTest] ERROR: Material list copy failed.\n";
        return false;
    }

    const bool countEqual =
        clone->matList.numMaterials == source->matList.numMaterials;

    const bool arrayValid =
        clone->matList.numMaterials == 0 ||
        clone->matList.materials != nullptr;

    const bool arrayIndependent =
        sourceMaterialCount == 0 ||
        clone->matList.materials != source->matList.materials;

    bool materialsEqual = countEqual && arrayValid;

    if (materialsEqual)
    {
        for (RwInt32 i = 0; i < sourceMaterialCount; ++i)
        {
            RpMaterial* sourceMaterial = source->matList.materials[i];
            RpMaterial* cloneMaterial  = clone->matList.materials[i];

            const bool same = sourceMaterial == cloneMaterial;

            log << "[CloneTest] material[" << i << "]"
                << " source=" << sourceMaterial
                << " clone=" << cloneMaterial
                << " equal=" << same << "\n";

            if (!same)
                materialsEqual = false;
        }
    }

    log << "[CloneTest] materialCountEqual=" << countEqual << "\n";
    log << "[CloneTest] materialArrayValid=" << arrayValid << "\n";
    log << "[CloneTest] materialArrayIndependent=" << arrayIndependent << "\n";
    log << "[CloneTest] materialsEqual=" << materialsEqual << "\n";

    OutputDebugStringA("[CloneTest] STEP D completed.\n");

    return countEqual && arrayValid && arrayIndependent && materialsEqual;
}

// ============================================================
// Atomic Geometry Attach / Restore Test
// ============================================================

static bool TestAtomicGeometryAttachRestore(
    RpAtomic* targetAtomic,
    RpGeometry* source,
    RpGeometry* clone,
    std::ostringstream& log,
    bool& cloneStillAttached)
{
    cloneStillAttached = false;

    log << "\n";
    log << "========== [AttachTest] STEP F ==========\n";

    if (!targetAtomic)
    {
        log << "[AttachTest] FAILED: target Atomic is null.\n";
        return false;
    }

    if (!source || !clone)
    {
        log << "[AttachTest] FAILED: source or clone is null.\n";
        return false;
    }

    RpGeometry* originalGeometry = RpAtomicGetGeometry(targetAtomic);
    RpHAnimHierarchy* originalHierarchy =
        RpSkinAtomicGetHAnimHierarchy(targetAtomic);

    log << "[AttachTest] targetAtomic=" << targetAtomic << "\n";
    log << "[AttachTest] originalGeometry=" << originalGeometry << "\n";
    log << "[AttachTest] expectedSourceGeometry=" << source << "\n";
    log << "[AttachTest] originalHierarchy=" << originalHierarchy << "\n";
    log << "[AttachTest] cloneGeometry=" << clone << "\n";

    if (originalGeometry != source)
    {
        log << "[AttachTest] FAILED: Atomic no longer uses source Geometry.\n";
        return false;
    }

    if (!originalHierarchy)
    {
        log << "[AttachTest] FAILED: Original HAnim hierarchy is null.\n";
        return false;
    }

    RpSkin* cloneSkin = RpSkinGeometryGetSkin(clone);
    if (!cloneSkin)
    {
        log << "[AttachTest] FAILED: Clone Skin is null.\n";
        return false;
    }

    log << "[AttachTest] BEFORE RpAtomicSetGeometry.\n";

    RpAtomic* attachResult = RpAtomicSetGeometry(targetAtomic, clone, 0);

    log << "[AttachTest] AFTER RpAtomicSetGeometry.\n";
    log << "[AttachTest] attachResult=" << attachResult << "\n";

    RpGeometry* geometryAfterAttach = RpAtomicGetGeometry(targetAtomic);
    RpHAnimHierarchy* hierarchyAfterAttach =
        RpSkinAtomicGetHAnimHierarchy(targetAtomic);

    RpSkin* skinAfterAttach = nullptr;
    if (geometryAfterAttach)
        skinAfterAttach = RpSkinGeometryGetSkin(geometryAfterAttach);

    const bool geometryAttached    = geometryAfterAttach == clone;
    const bool hierarchyPreserved  = hierarchyAfterAttach == originalHierarchy;
    const bool skinPreserved       = skinAfterAttach == cloneSkin;

    log << "[AttachTest] geometryAfterAttach=" << geometryAfterAttach << "\n";
    log << "[AttachTest] hierarchyAfterAttach=" << hierarchyAfterAttach << "\n";
    log << "[AttachTest] skinAfterAttach=" << skinAfterAttach << "\n";
    log << "[AttachTest] geometryAttached=" << geometryAttached << "\n";
    log << "[AttachTest] hierarchyPreserved=" << hierarchyPreserved << "\n";
    log << "[AttachTest] skinPreserved=" << skinPreserved << "\n";

    if (geometryAfterAttach != originalGeometry)
    {
        log << "[AttachTest] Atomic Geometry changed.\n";
        log << "[AttachTest] BEFORE restoring original Geometry.\n";

        RpAtomic* restoreResult =
            RpAtomicSetGeometry(targetAtomic, originalGeometry, 0);

        log << "[AttachTest] AFTER restoring original Geometry.\n";
        log << "[AttachTest] restoreResult=" << restoreResult << "\n";
    }
    else
    {
        log << "[AttachTest] Atomic still uses original Geometry; "
            << "restore call skipped.\n";
    }

    RpGeometry* geometryAfterRestore = RpAtomicGetGeometry(targetAtomic);
    RpHAnimHierarchy* hierarchyAfterRestore =
        RpSkinAtomicGetHAnimHierarchy(targetAtomic);

    cloneStillAttached = geometryAfterRestore == clone;

    const bool geometryRestored  = geometryAfterRestore == originalGeometry;
    const bool hierarchyRestored = hierarchyAfterRestore == originalHierarchy;

    log << "[AttachTest] originalGeometry=" << originalGeometry << "\n";
    log << "[AttachTest] geometryAfterRestore=" << geometryAfterRestore << "\n";
    log << "[AttachTest] hierarchyAfterRestore=" << hierarchyAfterRestore << "\n";
    log << "[AttachTest] geometryRestored=" << geometryRestored << "\n";
    log << "[AttachTest] originalGeometryRestored="
        << (geometryRestored ? "YES" : "NO") << "\n";
    log << "[AttachTest] hierarchyRestored=" << hierarchyRestored << "\n";
    log << "[AttachTest] cloneStillAttached=" << cloneStillAttached << "\n";

    const bool passed =
        geometryAttached &&
        hierarchyPreserved &&
        skinPreserved &&
        geometryRestored &&
        hierarchyRestored &&
        !cloneStillAttached;

    log << "[AttachTest] RESULT=" << (passed ? "SUCCESS" : "FAILED") << "\n";
    log << "========================================\n";

    return passed;
}

// ============================================================
// STEP H: Single Vertex Write / Readback / Restore Test
// ============================================================

static bool TestSingleVertexWriteRestore(
    RpGeometry* source,
    RpGeometry* clone,
    std::ostringstream& log)
{
    log << "\n";
    log << "========== [VertexTest] STEP H ==========\n";

    if (!source || !clone)
    {
        log << "[VertexTest] FAILED: source or clone is null.\n";
        return false;
    }

    if (source->numVertices <= 0 ||
        clone->numVertices <= 0 ||
        source->numMorphTargets <= 0 ||
        clone->numMorphTargets <= 0)
    {
        log << "[VertexTest] FAILED: Invalid geometry dimensions.\n";
        return false;
    }

    if (!(source->flags & rpGEOMETRYPOSITIONS) ||
        !(clone->flags & rpGEOMETRYPOSITIONS))
    {
        log << "[VertexTest] FAILED: Position data is unavailable.\n";
        return false;
    }

    if (!source->morphTarget ||
        !clone->morphTarget ||
        !source->morphTarget[0].verts ||
        !clone->morphTarget[0].verts)
    {
        log << "[VertexTest] FAILED: Vertex array is null.\n";
        return false;
    }

    if (source->numVertices != clone->numVertices)
    {
        log << "[VertexTest] FAILED: Vertex count mismatch.\n";
        return false;
    }

    const RwV3d sourceOriginal = source->morphTarget[0].verts[0];
    RwV3d originalVertex = {};
    const RwReal offset = 0.01f;

    log << "[VertexTest] H-1: Locking clone Geometry.\n";

    if (!RpGeometryLock(clone, rpGEOMETRYLOCKALL))
    {
        log << "[VertexTest] FAILED: Initial GeometryLock failed.\n";
        return false;
    }

    originalVertex = clone->morphTarget[0].verts[0];

    log << "[VertexTest] original vertex[0]=("
        << originalVertex.x << ", "
        << originalVertex.y << ", "
        << originalVertex.z << ")\n";

    clone->morphTarget[0].verts[0].x = originalVertex.x + offset;

    const RwV3d expectedModified = {
        originalVertex.x + offset,
        originalVertex.y,
        originalVertex.z
    };

    log << "[VertexTest] modified vertex[0]=("
        << clone->morphTarget[0].verts[0].x << ", "
        << clone->morphTarget[0].verts[0].y << ", "
        << clone->morphTarget[0].verts[0].z << ")\n";

    RpGeometryUnlock(clone);

    log << "[VertexTest] H-2: Geometry unlocked after modification.\n";
    log << "[VertexTest] H-3: Locking clone for readback.\n";

    if (!RpGeometryLock(clone, rpGEOMETRYLOCKALL))
    {
        log << "[VertexTest] ERROR: Readback GeometryLock failed.\n";
        log << "[VertexTest] Attempting emergency vertex restoration.\n";

        if (RpGeometryLock(clone, rpGEOMETRYLOCKALL))
        {
            clone->morphTarget[0].verts[0] = originalVertex;
            RpGeometryUnlock(clone);
            log << "[VertexTest] Emergency restoration attempted.\n";
        }
        else
        {
            log << "[VertexTest] WARNING: Emergency restoration Lock failed.\n";
        }

        return false;
    }

    const RwV3d modifiedReadback = clone->morphTarget[0].verts[0];

    const bool modificationCorrect =
        modifiedReadback.x == expectedModified.x &&
        modifiedReadback.y == expectedModified.y &&
        modifiedReadback.z == expectedModified.z;

    log << "[VertexTest] readback vertex[0]=("
        << modifiedReadback.x << ", "
        << modifiedReadback.y << ", "
        << modifiedReadback.z << ")\n";

    log << "[VertexTest] modificationCorrect=" << modificationCorrect << "\n";

    clone->morphTarget[0].verts[0] = originalVertex;

    log << "[VertexTest] original vertex restored in memory.\n";

    RpGeometryUnlock(clone);

    log << "[VertexTest] H-4: Geometry unlocked after restoration.\n";
    log << "[VertexTest] H-5: Locking clone for final verification.\n";

    if (!RpGeometryLock(clone, rpGEOMETRYLOCKALL))
    {
        log << "[VertexTest] FAILED: Final GeometryLock failed.\n";
        return false;
    }

    const RwV3d restoredReadback = clone->morphTarget[0].verts[0];

    const bool restorationCorrect =
        restoredReadback.x == originalVertex.x &&
        restoredReadback.y == originalVertex.y &&
        restoredReadback.z == originalVertex.z;

    log << "[VertexTest] restored vertex[0]=("
        << restoredReadback.x << ", "
        << restoredReadback.y << ", "
        << restoredReadback.z << ")\n";

    log << "[VertexTest] restorationCorrect=" << restorationCorrect << "\n";

    RpGeometryUnlock(clone);

    const RwV3d sourceAfter = source->morphTarget[0].verts[0];

    const bool sourceUnchanged =
        sourceAfter.x == sourceOriginal.x &&
        sourceAfter.y == sourceOriginal.y &&
        sourceAfter.z == sourceOriginal.z;

    log << "[VertexTest] source vertex before=("
        << sourceOriginal.x << ", "
        << sourceOriginal.y << ", "
        << sourceOriginal.z << ")\n";

    log << "[VertexTest] source vertex after=("
        << sourceAfter.x << ", "
        << sourceAfter.y << ", "
        << sourceAfter.z << ")\n";

    log << "[VertexTest] sourceUnchanged=" << sourceUnchanged << "\n";

    const bool passed =
        modificationCorrect && restorationCorrect && sourceUnchanged;

    log << "[VertexTest] RESULT=" << (passed ? "SUCCESS" : "FAILED") << "\n";
    log << "========================================\n";

    return passed;
}

// ============================================================
// Full Geometry Clone Test
// ============================================================

bool RunGeometryStreamCloneTest(
    RpAtomic* targetAtomic,
    RpGeometry* source,
    std::string& report)
{
    report.clear();

    std::ostringstream log;

    OutputDebugStringA("[CloneTest] Full geometry clone test entered.\n");

    if (!targetAtomic)
    {
        log << "[CloneTest] ERROR: target Atomic is null.\n";
        report = log.str();
        return false;
    }

    if (!source)
    {
        log << "[CloneTest] ERROR: source is null.\n";
        report = log.str();
        return false;
    }

    RpGeometry* initialAtomicGeometry = RpAtomicGetGeometry(targetAtomic);

    if (initialAtomicGeometry != source)
    {
        log << "[CloneTest] ERROR: target Atomic does not use source Geometry.\n";
        log << "[CloneTest] target Atomic Geometry=" << initialAtomicGeometry << "\n";
        log << "[CloneTest] source Geometry=" << source << "\n";
        report = log.str();
        return false;
    }

    log << "[CloneTest] target Atomic=" << targetAtomic << "\n";
    log << "[CloneTest] Source geometry=" << source << "\n";
    log << "[CloneTest] Source vertices=" << source->numVertices << "\n";
    log << "[CloneTest] Source triangles=" << source->numTriangles << "\n";
    log << "[CloneTest] Source morphTargets=" << source->numMorphTargets << "\n";
    log << "[CloneTest] Source texCoordSets=" << source->numTexCoordSets << "\n";
    log << "[CloneTest] Source flags=0x"
        << std::hex << source->flags << std::dec << "\n";

    if (source->numVertices <= 0 ||
        source->numTriangles < 0 ||
        source->numMorphTargets <= 0 ||
        source->numTexCoordSets < 0)
    {
        log << "[CloneTest] ERROR: Invalid source geometry dimensions.\n";
        report = log.str();
        return false;
    }

    if (!(source->flags & rpGEOMETRYPOSITIONS))
    {
        log << "[CloneTest] ERROR: Source geometry has no position data.\n";
        report = log.str();
        return false;
    }

    OutputDebugStringA("[CloneTest] BEFORE RpGeometryCreate.\n");

    RpGeometry* clone = RpGeometryCreate(
        source->numVertices,
        source->numTriangles,
        source->flags
    );

    OutputDebugStringA("[CloneTest] AFTER RpGeometryCreate.\n");

    if (!clone)
    {
        log << "[CloneTest] ERROR: RpGeometryCreate failed.\n";
        report = log.str();
        return false;
    }

    log << "[CloneTest] Clone geometry=" << clone << "\n";

    bool copyValid = true;
    bool geometryLocked = false;
    bool skinAttached = false;
    RpSkin* createdSkin = nullptr;

    bool attachTestPassed = false;
    bool cloneStillAttached = false;
    bool vertexTestPassed = false;

    DumpGeometryMeshDiag("DIAG 1 - SOURCE AFTER RpGeometryCreate", source, log);
    DumpGeometryMeshDiag("DIAG 1 - CLONE AFTER RpGeometryCreate", clone, log);

    if (source->numVertices != clone->numVertices ||
        source->numTriangles != clone->numTriangles ||
        source->numMorphTargets != clone->numMorphTargets ||
        source->numTexCoordSets != clone->numTexCoordSets)
    {
        log << "[CloneTest] ERROR: Geometry dimensions mismatch.\n";
        copyValid = false;
    }

    if (source->flags != clone->flags)
    {
        log << "[CloneTest] ERROR: Geometry flags mismatch.\n";
        copyValid = false;
    }

    if (copyValid)
    {
        if (source->numTriangles > 0)
        {
            copyValid =
                CheckPointer(source->triangles, "source triangles", log) &&
                CheckPointer(clone->triangles, "clone triangles", log);
        }

        copyValid =
            copyValid &&
            CheckPointer(source->morphTarget, "source morphTarget", log) &&
            CheckPointer(clone->morphTarget, "clone morphTarget", log);
    }

    if (copyValid && (source->flags & rpGEOMETRYPRELIT))
    {
        copyValid =
            CheckPointer(source->preLitLum, "source preLitLum", log) &&
            CheckPointer(clone->preLitLum, "clone preLitLum", log);
    }

    if (copyValid && (source->flags & rpGEOMETRYPOSITIONS))
    {
        copyValid =
            CheckPointer(source->morphTarget[0].verts, "source vertices", log) &&
            CheckPointer(clone->morphTarget[0].verts, "clone vertices", log);
    }

    if (copyValid && (source->flags & rpGEOMETRYNORMALS))
    {
        copyValid =
            CheckPointer(source->morphTarget[0].normals, "source normals", log) &&
            CheckPointer(clone->morphTarget[0].normals, "clone normals", log);
    }

    for (RwInt32 i = 0; copyValid && i < source->numTexCoordSets; ++i)
    {
        if (!source->texCoords[i] || !clone->texCoords[i])
        {
            log << "[CloneTest] ERROR: UV set " << i << " is null.\n";
            copyValid = false;
        }
    }

    if (copyValid)
    {
        if (source->numTriangles > 0)
        {
            copyValid = CheckIndependentPointer(
                source->triangles, clone->triangles, "triangleArray", log);
        }

        copyValid = copyValid &&
            CheckIndependentPointer(
                source->morphTarget, clone->morphTarget, "morphTargetArray", log);

        if (copyValid && (source->flags & rpGEOMETRYPOSITIONS))
        {
            copyValid = CheckIndependentPointer(
                source->morphTarget[0].verts,
                clone->morphTarget[0].verts,
                "vertexArray", log);
        }

        if (copyValid && (source->flags & rpGEOMETRYNORMALS))
        {
            copyValid = CheckIndependentPointer(
                source->morphTarget[0].normals,
                clone->morphTarget[0].normals,
                "normalArray", log);
        }

        if (copyValid && (source->flags & rpGEOMETRYPRELIT))
        {
            copyValid = CheckIndependentPointer(
                source->preLitLum, clone->preLitLum, "preLitArray", log);
        }

        for (RwInt32 i = 0; copyValid && i < source->numTexCoordSets; ++i)
        {
            std::ostringstream name;
            name << "uvArray[" << i << "]";

            copyValid = CheckIndependentPointer(
                source->texCoords[i],
                clone->texCoords[i],
                name.str().c_str(),
                log);
        }
    }

    if (copyValid)
    {
        copyValid = CopyGeometryMaterialList(source, clone, log);
    }

    if (copyValid)
    {
        OutputDebugStringA("[CloneTest] BEFORE RpGeometryLock.\n");

        RpGeometry* locked = RpGeometryLock(clone, rpGEOMETRYLOCKALL);

        OutputDebugStringA("[CloneTest] AFTER RpGeometryLock.\n");

        if (!locked)
        {
            log << "[CloneTest] ERROR: RpGeometryLock failed.\n";
            copyValid = false;
        }
        else
        {
            geometryLocked = true;
        }
    }

    if (copyValid && geometryLocked)
    {
        OutputDebugStringA("[CloneTest] BEFORE memcpy geometry arrays.\n");

        const size_t vertexCount = static_cast<size_t>(source->numVertices);
        const size_t triangleCount = static_cast<size_t>(source->numTriangles);

        if (triangleCount > 0)
        {
            std::memcpy(
                clone->triangles,
                source->triangles,
                sizeof(RpTriangle) * triangleCount);
        }

        if (source->flags & rpGEOMETRYPRELIT)
        {
            std::memcpy(
                clone->preLitLum,
                source->preLitLum,
                sizeof(RwRGBA) * vertexCount);
        }

        std::memcpy(
            clone->morphTarget[0].verts,
            source->morphTarget[0].verts,
            sizeof(RwV3d) * vertexCount);

        if (source->flags & rpGEOMETRYNORMALS)
        {
            std::memcpy(
                clone->morphTarget[0].normals,
                source->morphTarget[0].normals,
                sizeof(RwV3d) * vertexCount);
        }

        for (RwInt32 i = 0; i < source->numTexCoordSets; ++i)
        {
            std::memcpy(
                clone->texCoords[i],
                source->texCoords[i],
                sizeof(RwTexCoords) * vertexCount);
        }

        clone->morphTarget[0].boundingSphere =
            source->morphTarget[0].boundingSphere;

        OutputDebugStringA("[CloneTest] AFTER memcpy geometry arrays.\n");

        DumpTriangleInfo("SOURCE BEFORE UNLOCK", source, log);
        DumpTriangleInfo("CLONE BEFORE UNLOCK", clone, log);
    }

    if (geometryLocked)
    {
        OutputDebugStringA("[CloneTest] BEFORE RpGeometryUnlock.\n");

        RpGeometryUnlock(clone);
        geometryLocked = false;

        OutputDebugStringA("[CloneTest] AFTER RpGeometryUnlock.\n");
    }

    DumpGeometryMeshDiag("DIAG 2 - SOURCE AFTER RpGeometryUnlock", source, log);
    DumpGeometryMeshDiag("DIAG 2 - CLONE AFTER RpGeometryUnlock", clone, log);

    if (copyValid)
    {
        OutputDebugStringA("[CloneTest] BEFORE geometry memory comparison.\n");

        const size_t vertexCount = static_cast<size_t>(source->numVertices);
        const size_t triangleCount = static_cast<size_t>(source->numTriangles);

        const bool trianglesEqual =
            triangleCount == 0 ||
            std::memcmp(
                source->triangles,
                clone->triangles,
                sizeof(RpTriangle) * triangleCount) == 0;

        const bool colorsEqual =
            !(source->flags & rpGEOMETRYPRELIT) ||
            std::memcmp(
                source->preLitLum,
                clone->preLitLum,
                sizeof(RwRGBA) * vertexCount) == 0;

        const bool verticesEqual =
            std::memcmp(
                source->morphTarget[0].verts,
                clone->morphTarget[0].verts,
                sizeof(RwV3d) * vertexCount) == 0;

        const bool normalsEqual =
            !(source->flags & rpGEOMETRYNORMALS) ||
            std::memcmp(
                source->morphTarget[0].normals,
                clone->morphTarget[0].normals,
                sizeof(RwV3d) * vertexCount) == 0;

        bool uvEqual = true;
        for (RwInt32 i = 0; i < source->numTexCoordSets; ++i)
        {
            if (std::memcmp(
                source->texCoords[i],
                clone->texCoords[i],
                sizeof(RwTexCoords) * vertexCount) != 0)
            {
                uvEqual = false;
                break;
            }
        }

        log << "[CloneTest] trianglesEqual=" << trianglesEqual << "\n";
        log << "[CloneTest] colorsEqual=" << colorsEqual << "\n";
        log << "[CloneTest] verticesEqual=" << verticesEqual << "\n";
        log << "[CloneTest] normalsEqual=" << normalsEqual << "\n";
        log << "[CloneTest] uvEqual=" << uvEqual << "\n";

        copyValid = trianglesEqual && colorsEqual && verticesEqual &&
                    normalsEqual && uvEqual;

        OutputDebugStringA("[CloneTest] AFTER geometry memory comparison.\n");
    }

    if (copyValid)
    {
        OutputDebugStringA("[CloneTest] STEP E: Skin copy entered.\n");

        RpSkin* sourceSkin = RpSkinGeometryGetSkin(source);

        log << "[CloneTest] source Skin=" << sourceSkin << "\n";

        if (!sourceSkin)
        {
            log << "[CloneTest] ERROR: Source Skin is null.\n";
            copyValid = false;
        }
        else
        {
            const RwUInt32 numBones = RpSkinGetNumBones(sourceSkin);
            const RwUInt32 numVertices = static_cast<RwUInt32>(source->numVertices);
            const RwBool sourceSplit = RpSkinIsSplit(sourceSkin);

            log << "[CloneTest] Skin vertices=" << numVertices << "\n";
            log << "[CloneTest] Skin bones=" << numBones << "\n";
            log << "[CloneTest] Skin split=" << static_cast<int>(sourceSplit) << "\n";

            if (numBones == 0)
            {
                log << "[CloneTest] ERROR: Skin has zero bones.\n";
                copyValid = false;
            }

            if (sourceSplit)
            {
                log << "[CloneTest] ERROR: Split Skin is not supported by this test.\n";
                copyValid = false;
            }

            const RwMatrixWeights* sourceWeights = nullptr;
            const RwUInt32* sourceIndices = nullptr;
            const RwMatrix* sourceMatrices = nullptr;

            if (copyValid)
            {
                sourceWeights = RpSkinGetVertexBoneWeights(sourceSkin);
                sourceIndices = RpSkinGetVertexBoneIndices(sourceSkin);
                sourceMatrices = RpSkinGetSkinToBoneMatrices(sourceSkin);

                copyValid =
                    CheckPointer(sourceWeights, "source Skin weights", log) &&
                    CheckPointer(sourceIndices, "source Skin indices", log) &&
                    CheckPointer(sourceMatrices, "source Skin inverse matrices", log);
            }

            if (copyValid)
            {
                OutputDebugStringA("[CloneTest] BEFORE allocating Skin buffers.\n");

                const size_t weightCount = static_cast<size_t>(numVertices);
                const size_t indexCount = static_cast<size_t>(numVertices) * 4;

                std::vector<RwMatrixWeights> weights(weightCount);
                std::vector<RwUInt32> indices(indexCount);
                std::vector<RwMatrix> inverseMatrices(static_cast<size_t>(numBones));

                std::memcpy(weights.data(), sourceWeights,
                    sizeof(RwMatrixWeights) * weightCount);
                std::memcpy(indices.data(), sourceIndices,
                    sizeof(RwUInt32) * indexCount);
                std::memcpy(inverseMatrices.data(), sourceMatrices,
                    sizeof(RwMatrix) * static_cast<size_t>(numBones));

                OutputDebugStringA("[CloneTest] AFTER copying Skin source arrays.\n");
                OutputDebugStringA("[CloneTest] BEFORE RpSkinCreate.\n");

                createdSkin = RpSkinCreate(
                    numVertices,
                    numBones,
                    weights.data(),
                    indices.data(),
                    inverseMatrices.data());

                OutputDebugStringA("[CloneTest] AFTER RpSkinCreate.\n");

                log << "[CloneTest] clone Skin created=" << createdSkin << "\n";

                if (!createdSkin)
                {
                    log << "[CloneTest] ERROR: RpSkinCreate failed.\n";
                    copyValid = false;
                }
                else
                {
                    OutputDebugStringA("[CloneTest] BEFORE RpSkinGeometrySetSkin.\n");

                    RpGeometry* setResult = RpSkinGeometrySetSkin(clone, createdSkin);

                    OutputDebugStringA("[CloneTest] AFTER RpSkinGeometrySetSkin.\n");

                    RpSkin* attachedSkin = RpSkinGeometryGetSkin(clone);
                    skinAttached = attachedSkin == createdSkin;

                    log << "[CloneTest] setResult=" << setResult << "\n";
                    log << "[CloneTest] attachedSkin=" << attachedSkin << "\n";
                    log << "[CloneTest] skinAttached=" << skinAttached << "\n";

                    if (!setResult || !skinAttached)
                    {
                        log << "[CloneTest] ERROR: RpSkinGeometrySetSkin failed.\n";
                        copyValid = false;
                    }
                    else
                    {
                        RpSkin* readBackSkin = RpSkinGeometryGetSkin(clone);
                        log << "[CloneTest] readback Skin=" << readBackSkin << "\n";

                        if (!readBackSkin)
                        {
                            log << "[CloneTest] ERROR: Clone Skin readback failed.\n";
                            copyValid = false;
                        }
                        else
                        {
                            const RwUInt32 cloneBoneCount =
                                RpSkinGetNumBones(readBackSkin);
                            const RwMatrixWeights* cloneWeights =
                                RpSkinGetVertexBoneWeights(readBackSkin);
                            const RwUInt32* cloneIndices =
                                RpSkinGetVertexBoneIndices(readBackSkin);
                            const RwMatrix* cloneMatrices =
                                RpSkinGetSkinToBoneMatrices(readBackSkin);

                            const bool skinPointerIndependent = readBackSkin != sourceSkin;
                            const bool boneCountEqual = cloneBoneCount == numBones;
                            const bool weightsPointerValid = cloneWeights != nullptr;
                            const bool indicesPointerValid = cloneIndices != nullptr;
                            const bool matricesPointerValid = cloneMatrices != nullptr;
                            const bool weightsPointerIndependent =
                                weightsPointerValid && cloneWeights != sourceWeights;
                            const bool indicesPointerIndependent =
                                indicesPointerValid && cloneIndices != sourceIndices;
                            const bool matricesPointerIndependent =
                                matricesPointerValid && cloneMatrices != sourceMatrices;

                            log << "[CloneTest] skinPointerIndependent="
                                << skinPointerIndependent << "\n";
                            log << "[CloneTest] boneCountEqual=" << boneCountEqual << "\n";
                            log << "[CloneTest] weightsPointerIndependent="
                                << weightsPointerIndependent << "\n";
                            log << "[CloneTest] indicesPointerIndependent="
                                << indicesPointerIndependent << "\n";
                            log << "[CloneTest] matricesPointerIndependent="
                                << matricesPointerIndependent << "\n";

                            bool weightsEqual = false;
                            bool indicesEqual = false;
                            bool matricesEqual = false;

                            if (weightsPointerValid && indicesPointerValid &&
                                matricesPointerValid && boneCountEqual)
                            {
                                weightsEqual = std::memcmp(
                                    sourceWeights, cloneWeights,
                                    sizeof(RwMatrixWeights) * weightCount) == 0;
                                indicesEqual = std::memcmp(
                                    sourceIndices, cloneIndices,
                                    sizeof(RwUInt32) * indexCount) == 0;
                                matricesEqual = std::memcmp(
                                    sourceMatrices, cloneMatrices,
                                    sizeof(RwMatrix) * static_cast<size_t>(numBones)) == 0;
                            }

                            log << "[CloneTest] weightsEqual=" << weightsEqual << "\n";
                            log << "[CloneTest] indicesEqual=" << indicesEqual << "\n";
                            log << "[CloneTest] matricesEqual=" << matricesEqual << "\n";

                            copyValid =
                                skinPointerIndependent &&
                                boneCountEqual &&
                                weightsPointerIndependent &&
                                indicesPointerIndependent &&
                                matricesPointerIndependent &&
                                weightsEqual && indicesEqual && matricesEqual;
                        }
                    }
                }
            }
        }

        OutputDebugStringA("[CloneTest] STEP E completed.\n");
    }

    if (copyValid)
    {
        vertexTestPassed = TestSingleVertexWriteRestore(source, clone, log);
    }
    else
    {
        log << "[VertexTest] STEP H skipped because clone validation failed.\n";
    }

    if (copyValid && vertexTestPassed)
    {
        attachTestPassed = TestAtomicGeometryAttachRestore(
            targetAtomic, source, clone, log, cloneStillAttached);
    }
    else
    {
        log << "[AttachTest] STEP F skipped because STEP C-E or STEP H failed.\n";
    }

    DumpGeometryMeshDiag("DIAG 3 - SOURCE BEFORE CLONE DESTROY", source, log);
    DumpGeometryMeshDiag("DIAG 3 - CLONE BEFORE DESTROY", clone, log);

    if (geometryLocked)
    {
        OutputDebugStringA("[CloneTest] WARNING: Unlocking clone during cleanup.\n");
        RpGeometryUnlock(clone);
        geometryLocked = false;
    }

    if (createdSkin && !skinAttached)
    {
        OutputDebugStringA("[CloneTest] Destroying unattached temporary Skin.\n");
        RpSkinDestroy(createdSkin);
        createdSkin = nullptr;
    }

    RwBool destroyResult = FALSE;

    if (cloneStillAttached)
    {
        log << "[CloneTest] WARNING: Clone is still attached to target Atomic.\n";
        log << "[CloneTest] Skipping RpGeometryDestroy to avoid dangling pointer.\n";
        OutputDebugStringA("[CloneTest] WARNING: clone still attached; destruction skipped.\n");
    }
    else
    {
        OutputDebugStringA("[CloneTest] BEFORE RpGeometryDestroy.\n");
        destroyResult = RpGeometryDestroy(clone);
        OutputDebugStringA("[CloneTest] AFTER RpGeometryDestroy.\n");
    }

    log << "[CloneTest] skinAttached=" << skinAttached << "\n";
    log << "[CloneTest] vertexTestPassed=" << vertexTestPassed << "\n";
    log << "[CloneTest] attachTestPassed=" << attachTestPassed << "\n";
    log << "[CloneTest] cloneStillAttached=" << cloneStillAttached << "\n";
    log << "[CloneTest] destroyResult=" << static_cast<int>(destroyResult) << "\n";

    const bool cloneDestroyed = !cloneStillAttached && destroyResult != 0;

    const bool testPassed =
        copyValid && vertexTestPassed && attachTestPassed && cloneDestroyed;

    log << "[CloneTest] STEP C " << (copyValid ? "PASSED" : "FAILED") << ".\n";
    log << "[CloneTest] STEP D " << (copyValid ? "PASSED" : "FAILED") << ".\n";
    log << "[CloneTest] STEP E " << (copyValid ? "PASSED" : "FAILED") << ".\n";
    log << "[CloneTest] STEP H " << (vertexTestPassed ? "PASSED" : "FAILED") << ".\n";
    log << "[CloneTest] STEP F " << (attachTestPassed ? "PASSED" : "FAILED") << ".\n";
    log << "[CloneTest] STEP G " << (cloneDestroyed ? "PASSED" : "FAILED") << ".\n";
    log << "[CloneTest] RESULT=" << (testPassed ? "SUCCESS" : "FAILED") << "\n";
    log << "[CloneTest] cloneDestroyed=" << (cloneDestroyed ? "YES" : "NO") << "\n";

    report = log.str();
    return testPassed;
}

#include <map>

// ============================================================
// STEP I: Persistent Geometry Vertex Test
// ============================================================

namespace
{
    constexpr int kStepITestFrames = 120;
    constexpr RwReal kStepIVertexOffset = 0.03f;

    struct PersistentVertexTest
    {
        RpGeometry* sourceGeometry = nullptr;
        RpGeometry* cloneGeometry = nullptr;
        RwInt32 vertexIndex = -1;
        RwV3d originalVertex{};
        int frames = 0;
        bool attached = false;
    };

    static std::map<RpAtomic*, PersistentVertexTest> g_persistentVertexTests;

    static void StepILog(std::ostringstream& log, const char* message)
    {
        log << "[StepI] " << message << "\n";
    }

    static void LogSourceVertex(
        std::ostringstream& log,
        const PersistentVertexTest& state)
    {
        if (!state.sourceGeometry ||
            !state.cloneGeometry ||
            !state.sourceGeometry->morphTarget ||
            !state.cloneGeometry->morphTarget ||
            state.vertexIndex < 0) {
            log << " sourceVertex=UNAVAILABLE\n";
            return;
        }

        const RwV3d& src = state.sourceGeometry->morphTarget[0].verts[state.vertexIndex];
        const RwV3d& clo = state.cloneGeometry->morphTarget[0].verts[state.vertexIndex];

        log << " sourceX=" << src.x
            << " sourceY=" << src.y
            << " sourceZ=" << src.z
            << " cloneX="  << clo.x
            << " cloneY="  << clo.y
            << " cloneZ="  << clo.z
            << "\n";
    }

    static RpGeometry* CreatePersistentGeometryClone(
        RpGeometry* source,
        std::ostringstream& log)
    {
        if (!source)
        {
            StepILog(log, "Source Geometry is null.");
            return nullptr;
        }

        if (source->numVertices <= 0 ||
            source->numTriangles < 0 ||
            source->numMorphTargets <= 0 ||
            source->numTexCoordSets < 0)
        {
            StepILog(log, "Invalid source Geometry dimensions.");
            return nullptr;
        }

        if (source->numMorphTargets != 1)
        {
            StepILog(log, "This test currently supports exactly one morph target.");
            return nullptr;
        }

        if (!(source->flags & rpGEOMETRYPOSITIONS) ||
            !source->morphTarget ||
            !source->morphTarget[0].verts)
        {
            StepILog(log, "Source vertex data is unavailable.");
            return nullptr;
        }

        if (source->numTriangles > 0 && !source->triangles)
        {
            StepILog(log, "Source triangle array is null.");
            return nullptr;
        }

        if ((source->flags & rpGEOMETRYPRELIT) && !source->preLitLum)
        {
            StepILog(log, "Source preLitLum is null.");
            return nullptr;
        }

        if ((source->flags & rpGEOMETRYNORMALS) &&
            !source->morphTarget[0].normals)
        {
            StepILog(log, "Source normal array is null.");
            return nullptr;
        }

        for (RwInt32 i = 0; i < source->numTexCoordSets; ++i)
        {
            if (!source->texCoords[i])
            {
                StepILog(log, "Source UV array is null.");
                return nullptr;
            }
        }

        RpGeometry* clone = RpGeometryCreate(
            source->numVertices,
            source->numTriangles,
            source->flags);

        if (!clone)
        {
            StepILog(log, "RpGeometryCreate failed.");
            return nullptr;
        }

        bool skinAttached = false;
        RpSkin* createdSkin = nullptr;

        if (!_rpMaterialListCopy(&clone->matList, &source->matList))
        {
            StepILog(log, "MaterialList copy failed.");
            RpGeometryDestroy(clone);
            return nullptr;
        }

        if (!RpGeometryLock(clone, rpGEOMETRYLOCKALL))
        {
            StepILog(log, "RpGeometryLock failed.");
            RpGeometryDestroy(clone);
            return nullptr;
        }

        const size_t vertexCount = static_cast<size_t>(source->numVertices);
        const size_t triangleCount = static_cast<size_t>(source->numTriangles);

        if (triangleCount > 0)
        {
            std::memcpy(clone->triangles, source->triangles,
                sizeof(RpTriangle) * triangleCount);
        }

        std::memcpy(clone->morphTarget[0].verts,
            source->morphTarget[0].verts,
            sizeof(RwV3d) * vertexCount);

        if (source->flags & rpGEOMETRYNORMALS)
        {
            std::memcpy(clone->morphTarget[0].normals,
                source->morphTarget[0].normals,
                sizeof(RwV3d) * vertexCount);
        }

        if (source->flags & rpGEOMETRYPRELIT)
        {
            std::memcpy(clone->preLitLum,
                source->preLitLum,
                sizeof(RwRGBA) * vertexCount);
        }

        for (RwInt32 i = 0; i < source->numTexCoordSets; ++i)
        {
            std::memcpy(clone->texCoords[i],
                source->texCoords[i],
                sizeof(RwTexCoords) * vertexCount);
        }

        clone->morphTarget[0].boundingSphere =
            source->morphTarget[0].boundingSphere;

        RpGeometryUnlock(clone);

        RpSkin* sourceSkin = RpSkinGeometryGetSkin(source);
        if (!sourceSkin)
        {
            StepILog(log, "Source Skin is null.");
            RpGeometryDestroy(clone);
            return nullptr;
        }

        const RwUInt32 boneCount = RpSkinGetNumBones(sourceSkin);
        const RwUInt32 skinVertexCount = static_cast<RwUInt32>(source->numVertices);

        if (boneCount == 0 || RpSkinIsSplit(sourceSkin))
        {
            StepILog(log, "Invalid Skin: zero bones or split Skin.");
            RpGeometryDestroy(clone);
            return nullptr;
        }

        const RwMatrixWeights* sourceWeights =
            RpSkinGetVertexBoneWeights(sourceSkin);
        const RwUInt32* sourceIndices =
            RpSkinGetVertexBoneIndices(sourceSkin);
        const RwMatrix* sourceMatrices =
            RpSkinGetSkinToBoneMatrices(sourceSkin);

        if (!sourceWeights || !sourceIndices || !sourceMatrices)
        {
            StepILog(log, "Source Skin arrays are invalid.");
            RpGeometryDestroy(clone);
            return nullptr;
        }

        std::vector<RwMatrixWeights> weights(static_cast<size_t>(skinVertexCount));
        std::vector<RwUInt32> indices(static_cast<size_t>(skinVertexCount) * 4);
        std::vector<RwMatrix> inverseMatrices(static_cast<size_t>(boneCount));

        std::memcpy(weights.data(), sourceWeights,
            sizeof(RwMatrixWeights) * static_cast<size_t>(skinVertexCount));
        std::memcpy(indices.data(), sourceIndices,
            sizeof(RwUInt32) * static_cast<size_t>(skinVertexCount) * 4);
        std::memcpy(inverseMatrices.data(), sourceMatrices,
            sizeof(RwMatrix) * static_cast<size_t>(boneCount));

        createdSkin = RpSkinCreate(
            skinVertexCount,
            boneCount,
            weights.data(),
            indices.data(),
            inverseMatrices.data());

        if (!createdSkin)
        {
            StepILog(log, "RpSkinCreate failed.");
            RpGeometryDestroy(clone);
            return nullptr;
        }

        RpGeometry* skinSetResult = RpSkinGeometrySetSkin(clone, createdSkin);
        skinAttached = RpSkinGeometryGetSkin(clone) == createdSkin;

        if (!skinSetResult || !skinAttached)
        {
            StepILog(log, "Attaching cloned Skin failed.");
            if (!skinAttached) RpSkinDestroy(createdSkin);
            RpGeometryDestroy(clone);
            return nullptr;
        }

        StepILog(log, "Persistent Geometry clone created.");
        log << "[StepI] sourceGeometry=" << source << "\n";
        log << "[StepI] cloneGeometry=" << clone << "\n";
        log << "[StepI] vertices=" << clone->numVertices << "\n";
        log << "[StepI] triangles=" << clone->numTriangles << "\n";
        log << "[StepI] Skin=" << RpSkinGeometryGetSkin(clone) << "\n";

        return clone;
    }

    // 去掉 alpha==255 检查
static RwInt32 FindMarkedHairVertex(
    RpGeometry* geometry,
    std::ostringstream& log)
{
    if (!geometry ||
        !geometry->preLitLum ||
        geometry->numVertices <= 0)
    {
        StepILog(log, "Cannot inspect physics vertex markers.");
        return -1;
    }

    // 用新三通道方案判断：R 是类型(0x40/0x80/0xC0/0xFF)，B 是锚定强度
    for (RwInt32 i = 0;
         i < geometry->numVertices;
         ++i)
    {
        const RwRGBA& color =
            geometry->preLitLum[i];

        VertexPhysParams params;
        const bool isPhysicsVertex =
            ClassifyVertexColor(color, params);

        if (isPhysicsVertex)
        {
            log << "[StepI] First marked physics vertex="
                << i
                << " RGBA=("
                << static_cast<int>(color.red) << ","
                << static_cast<int>(color.green) << ","
                << static_cast<int>(color.blue) << ","
                << static_cast<int>(color.alpha) << ")\n";

            return i;
        }
    }

    StepILog(log, "No marked physics vertex found.");
    return -1;
}

    static bool RestorePersistentVertexTest(
        RpAtomic* atomic,
        PersistentVertexTest& state,
        std::ostringstream& log)
    {
        if (!atomic || !state.sourceGeometry || !state.cloneGeometry)
        {
            StepILog(log, "Restore failed: invalid state.");
            return false;
        }

        RpGeometry* current = RpAtomicGetGeometry(atomic);

        if (current != state.cloneGeometry && current != state.sourceGeometry)
        {
            StepILog(log, "Atomic Geometry was replaced externally; refusing to overwrite it.");
            return false;
        }

        if (current == state.cloneGeometry)
        {
            if (RpGeometryLock(state.cloneGeometry, rpGEOMETRYLOCKALL))
            {
                state.cloneGeometry->morphTarget[0].verts[state.vertexIndex] =
                    state.originalVertex;
                RpGeometryUnlock(state.cloneGeometry);
                StepILog(log, "Clone vertex restored.");
            }
            else
            {
                StepILog(log, "Could not lock clone for vertex restoration.");
            }

            RpAtomic* result = RpAtomicSetGeometry(
                atomic, state.sourceGeometry, 0);

            if (!result)
            {
                StepILog(log, "RpAtomicSetGeometry restore returned null.");
                return false;
            }
        }

        const bool detached = RpAtomicGetGeometry(atomic) != state.cloneGeometry;

        if (!detached)
        {
            StepILog(log, "Clone is still attached; destruction skipped.");
            return false;
        }

        const RwBool destroyed = RpGeometryDestroy(state.cloneGeometry);

        log << "[StepI] RpGeometryDestroy=" << static_cast<int>(destroyed) << "\n";

        if (!destroyed)
        {
            StepILog(log, "Clone destruction failed.");
            return false;
        }

        state.cloneGeometry = nullptr;
        state.attached = false;
        StepILog(log, "Original Geometry restored; clone destroyed.");
        return true;
    }
}

bool IsGeometryVertexTestActive(RpAtomic* atomic)
{
    if (!atomic) return false;
    return g_persistentVertexTests.find(atomic) != g_persistentVertexTests.end();
}

bool BeginGeometryVertexTest(
    RpAtomic* atomic,
    std::string& report)
{
    report.clear();
    std::ostringstream log;

    if (!atomic)
    {
        StepILog(log, "Begin failed: Atomic is null.");
        report = log.str();
        return false;
    }

    if (IsGeometryVertexTestActive(atomic))
    {
        StepILog(log, "Test is already active.");
        report = log.str();
        return true;
    }

    RpGeometry* source = RpAtomicGetGeometry(atomic);

    if (!source)
    {
        StepILog(log, "Begin failed: source Geometry is null.");
        report = log.str();
        return false;
    }

    if (!(source->flags & rpGEOMETRYPRELIT))
    {
        StepILog(log, "Begin failed: Geometry has no prelight data.");
        report = log.str();
        return false;
    }

    RpHAnimHierarchy* hierarchy = RpSkinAtomicGetHAnimHierarchy(atomic);

    if (!hierarchy)
    {
        StepILog(log, "Begin failed: HAnim hierarchy is null.");
        report = log.str();
        return false;
    }

    const RwInt32 vertexIndex = FindMarkedHairVertex(source, log);

    if (vertexIndex < 0)
    {
        report = log.str();
        return false;
    }

    RpGeometry* clone = CreatePersistentGeometryClone(source, log);

    if (!clone)
    {
        report = log.str();
        return false;
    }

    PersistentVertexTest state;

    state.sourceGeometry = source;
    state.cloneGeometry  = clone;
    state.vertexIndex    = vertexIndex;
    state.originalVertex = clone->morphTarget[0].verts[vertexIndex];

    RpAtomic* attachResult = RpAtomicSetGeometry(atomic, clone, 0);

    if (!attachResult || RpAtomicGetGeometry(atomic) != clone)
    {
        StepILog(log, "Clone attach failed.");

        if (RpAtomicGetGeometry(atomic) != clone)
            RpGeometryDestroy(clone);

        report = log.str();
        return false;
    }

    state.attached = true;
    g_persistentVertexTests.emplace(atomic, state);

    StepILog(log, "Clone attached successfully.");

    log << "[StepI] atomic=" << atomic << "\n";
    log << "[StepI] sourceGeometry=" << source << "\n";
    log << "[StepI] cloneGeometry=" << clone << "\n";
    log << "[StepI] vertexIndex=" << vertexIndex << "\n";

    report = log.str();
    return true;
}

bool UpdateGeometryVertexTest(
    RpAtomic* atomic,
    std::string& report)
{
    report.clear();
    if (!atomic) return false;

    auto it = g_persistentVertexTests.find(atomic);
    if (it == g_persistentVertexTests.end()) return false;

    PersistentVertexTest& state = it->second;
    std::ostringstream log;

    if (!state.cloneGeometry || !state.sourceGeometry || state.vertexIndex < 0)
    {
        StepILog(log, "Invalid persistent test state.");
        report = log.str();
        return false;
    }

    if (RpAtomicGetGeometry(atomic) != state.cloneGeometry)
    {
        StepILog(log, "Atomic no longer uses test clone; attempting safe restore.");

        const bool restored = RestorePersistentVertexTest(atomic, state, log);

        if (restored)
            g_persistentVertexTests.erase(it);

        report = log.str();
        return false;
    }

    ++state.frames;

    if (!kPersistentPhysics)
    {
        if (!RpGeometryLock(state.cloneGeometry, rpGEOMETRYLOCKALL))
        {
            StepILog(log, "Per-frame GeometryLock failed.");
            report = log.str();
            return false;
        }

        RwV3d& vertex = state.cloneGeometry->morphTarget[0].verts[state.vertexIndex];

        vertex.x = state.originalVertex.x + kStepIVertexOffset;
        vertex.y = state.originalVertex.y;
        vertex.z = state.originalVertex.z;

        RpGeometryUnlock(state.cloneGeometry);

        if (state.frames == 1 || state.frames % 30 == 0)
        {
            log << "[StepI] frame=" << state.frames
                << " vertexIndex=" << state.vertexIndex
                << " vertex=("
                << vertex.x << "," << vertex.y << "," << vertex.z << ")";

            LogSourceVertex(log, state);
        }

        if (state.frames >= kStepITestFrames)
        {
            StepILog(log, "120-frame test completed.");

            const bool restored = RestorePersistentVertexTest(atomic, state, log);

            if (restored)
            {
                g_persistentVertexTests.erase(it);
                StepILog(log, "STEP I finished successfully.");
            }
            else
            {
                StepILog(log, "STEP I cleanup incomplete; state retained for retry.");
            }
        }
    }
    else
    {
        if (state.frames == 1 || state.frames % 60 == 0)
        {
            log << "[StepI] frame=" << state.frames
                << " vertexIndex=" << state.vertexIndex
                << " mode=PERSISTENT";

            LogSourceVertex(log, state);
        }
    }

    report = log.str();
    return true;
}