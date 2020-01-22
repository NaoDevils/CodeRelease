@ECHO OFF
:: Copy windows specific files into the .vscode folder in the projects root dir if the
:: files do not already exist

:: Execute /Make/VS2019/generate.cmd to create build targets
CALL ../VS2019/generate.cmd

:: create .vscode folder
IF EXIST "../../.vscode" (
    ECHO .vscode folder already exists, so creation is unnecessary.
) ELSE (
    ECHO Creating .vscode folder in project root directory...
    MKDIR "../../.vscode"
)

:: copy tasks.json
IF EXIST "../../.vscode/tasks.json" (
    ECHO WARNING: tasks.json  already exists, file will not be copied to avoid overwriting.
) ELSE (
    ECHO Copying tasks.json into .vscode folder...
    COPY "tasks_win.json" "../../.vscode/tasks.json"
)

:: copy launch_win.json
IF EXIST "../../.vscode/launch.json" (
    ECHO WARNING: launch.json  already exists, file will not be copied to avoid overwriting.
) ELSE (
    ECHO Copying launch_win.json into .vscode folder...
    :: rename file while copying so vscode can find it
    COPY "launch_win.json" "../../.vscode/launch.json"
)