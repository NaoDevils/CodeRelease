import os
import re

# Ignores modules
ignore = ["BehaviorControl.cpp", "BallchaserProvider.cpp"]


def removeComments(content):
    return re.sub(r"/\*.*?\*/|//.*?$", "", content, 0, re.MULTILINE | re.DOTALL)


def getStructs(content):
    return re.findall(
        r"(?:STREAMABLE\(|STREAMABLE_WITH_BASE\(|struct +|class +)(\w+)",
        removeComments(content),
        re.MULTILINE,
    )


def getDependencies(type, content):
    return re.findall(type + r"\((\w+)\)", content)


def removeDependency(type, name, content):
    # Regex also handles some edge cases with comments, see https://regex101.com/r/qRJbeN/2
    return re.sub(
        r",([^,]*)[\r\n]+\s*" + type + r"\(" + name + r"\)(,?).*",
        r"\2\1",
        content,
        0,
        re.MULTILINE,
    )


def getIncludedRepresentations(content):
    return re.findall(
        r"^#include [\"<]Representations[\w\/]*\/(\w+\.h)[\">]",
        content,
        re.MULTILINE,
    )


def removeIncludedRepresentation(name, content):
    return re.sub(
        r"^#include [\"<]Representations[\w\/]*\/("
        + name.replace(".", "\\.")
        + r")[\">].*[\r\n]+",
        "",
        content,
        0,
        re.MULTILINE,
    )


def isRepresentationUsed(name, content):
    return re.search(r"\bthe" + name + r"\b", content)


print("Reading representations")
headerToRep = {}
for root, dirs, files in os.walk("Src/Representations"):
    for file in files:
        if file.endswith(".h"):
            path = os.path.join(root, file)

            print(f"File: {path}")

            with open(path, "r", newline="") as f:
                content = f.read()
                headerToRep[file] = getStructs(removeComments(content))
                print(f"Representations: {headerToRep[file]}")


print("#" * 30)
print("#" * 30)


print("Reading modules")
for root, dirs, files in os.walk("Src/Modules"):
    for file in files:
        if file.endswith(".cpp"):
            print("-" * 30)

            header = os.path.join(root, file.replace(".cpp", ".h"))
            impl = os.path.join(root, file)
            print(f"File: {impl}")

            if any(element == file for element in ignore):
                print("Skip! File ignored!")
                continue

            try:
                with open(header, "r", newline="") as f:
                    headerContent = f.read()
                    filteredHeaderContent = removeComments(headerContent)

                    if filteredHeaderContent.find("MODULE(") == -1:
                        print("Skip! Not a module.")
                        continue

                    requires = getDependencies("REQUIRES", filteredHeaderContent)
                    uses = getDependencies("USES", filteredHeaderContent)
                    provides = getDependencies("PROVIDES[A-Z_]*", filteredHeaderContent)

                    includes = getIncludedRepresentations(filteredHeaderContent)
            except FileNotFoundError as error:
                print("Cannot open header")
                continue

            print(f"Requires: {requires}")
            print(f"Uses: {uses}")
            print(f"Provides: {provides}")

            unusedRequires = []
            unusedUses = []

            with open(impl, "r", newline="") as f:
                implContent = f.read()
                filteredImplContent = removeComments(implContent)

                for require in requires:
                    if (
                        not isRepresentationUsed(require, filteredImplContent)
                        and not isRepresentationUsed(require, filteredHeaderContent)
                        # special case, some modules use this to force update() order
                        and not require in provides
                    ):
                        unusedRequires.append(require)

                for use in uses:
                    if not isRepresentationUsed(
                        use, filteredImplContent
                    ) and not isRepresentationUsed(use, filteredHeaderContent):
                        unusedUses.append(use)

            for ur in unusedRequires:
                requires.remove(ur)
            for uu in unusedUses:
                uses.remove(uu)

            def isIncludeUnused(include):
                reps = headerToRep[include]
                return (
                    not any(rep in requires for rep in reps)
                    and not any(rep in uses for rep in reps)
                    and not any(rep in provides for rep in reps)
                )

            unusedIncludes = list(filter(isIncludeUnused, includes))

            print(f"Unused REQUIRES: {unusedRequires}")
            print(f"Unused USES: {unusedUses}")
            print(f"Unused #include: {unusedIncludes}")

            if unusedUses or unusedRequires or unusedIncludes:
                with open(header, "w", newline="") as f:
                    for ur in unusedRequires:
                        headerContent = removeDependency("REQUIRES", ur, headerContent)

                    for uu in unusedUses:
                        headerContent = removeDependency("USES", uu, headerContent)

                    for ui in unusedIncludes:
                        headerContent = removeIncludedRepresentation(ui, headerContent)

                    f.write(headerContent)
