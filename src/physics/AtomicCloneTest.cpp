#include "AtomicCloneTest.h"

#include "rpworld.h"

#include <sstream>
#include <iomanip>

static bool g_atomicCloneTestAttempted = false;

static RpAtomic* g_retainedAtomicClone = nullptr;


static int CountMarkedHairVertices(RpGeometry* geometry)
{
    if (!geometry ||
        !geometry->preLitLum ||
        geometry->numVertices <= 0)
    {
        return 0;
    }

    int count = 0;

    for (RwInt32 i = 0;
         i < geometry->numVertices;
         ++i)
    {
        const RwRGBA& color =
            geometry->preLitLum[i];

        const bool isPhysicsVertex =
            color.blue == 0 &&
            (color.red == 3 ||
             color.red == 255 ||
             color.red == 128);

        if (isPhysicsVertex)
        {
            ++count;
        }
    }

    return count;
}


static void AppendPointer(
    std::ostringstream& out,
    const char* name,
    const void* pointer)
{
    out << name
        << "=0x"
        << std::hex
        << reinterpret_cast<std::uintptr_t>(pointer)
        << std::dec;
}


bool RunAtomicCloneReadOnlyTest(
    RpAtomic* source,
    std::string& report)
{
    report.clear();

    if (g_atomicCloneTestAttempted)
    {
        return false;
    }

    if (!source)
    {
        return false;
    }

    RpGeometry* sourceGeometry =
        RpAtomicGetGeometry(source);

    if (!sourceGeometry)
    {
        return false;
    }

    const int hairVertexCount =
        CountMarkedHairVertices(sourceGeometry);

    if (hairVertexCount <= 0)
    {
        return false;
    }

    g_atomicCloneTestAttempted = true;

    RwFrame* sourceFrame =
        RpAtomicGetFrameMacro(source);

    RpClump* sourceClump =
        RpAtomicGetClumpMacro(source);

    std::ostringstream out;

    out << "========== Atomic Clone Read-Only Test ==========\n";

    out << "[AtomicCloneTest]"
        << " hairVertices=" << hairVertexCount
        << "\n";

    AppendPointer(out, "sourceAtomic", source);
    out << "\n";

    AppendPointer(out, "sourceGeometry", sourceGeometry);
    out << "\n";

    AppendPointer(out, "sourceFrame", sourceFrame);
    out << "\n";

    AppendPointer(out, "sourceClump", sourceClump);
    out << "\n";

    RpAtomic* clone = RpAtomicClone(source);

    if (!clone)
    {
        out << "[AtomicCloneTest] RpAtomicClone returned NULL.\n";
        out << "[AtomicCloneTest] RESULT=FAILED\n";
        report = out.str();
        return false;
    }

    g_retainedAtomicClone = clone;

    RpGeometry* cloneGeometry = RpAtomicGetGeometry(clone);
    RwFrame* cloneFrame = RpAtomicGetFrameMacro(clone);
    RpClump* cloneClump = RpAtomicGetClumpMacro(clone);

    out << "\n[AtomicCloneResult]\n";

    AppendPointer(out, "cloneAtomic", clone);
    out << "\n";

    AppendPointer(out, "cloneGeometry", cloneGeometry);
    out << "\n";

    AppendPointer(out, "cloneFrame", cloneFrame);
    out << "\n";

    AppendPointer(out, "cloneClump", cloneClump);
    out << "\n";

    out << std::boolalpha;

    out << "[AtomicCloneComparison]"
        << " sameGeometry="
        << (sourceGeometry == cloneGeometry)
        << " sameFrame="
        << (sourceFrame == cloneFrame)
        << " sameClump="
        << (sourceClump == cloneClump)
        << "\n";

    out << "[AtomicCloneTest]"
        << " retainedClone=YES"
        << " destroyCalled=NO"
        << " sourceModified=NO"
        << "\n";

    out << "[AtomicCloneTest] RESULT=CLONE_CREATED\n";
    out << "==================================================";

    report = out.str();
    return true;
}