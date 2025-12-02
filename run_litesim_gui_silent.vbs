' Run LiteSim without showing a terminal window (uses the batch launcher).
Option Explicit
Dim shell, scriptPath
Set shell = CreateObject("WScript.Shell")
scriptPath = Replace(WScript.ScriptFullName, WScript.ScriptName, "run_litesim_gui.bat")
shell.Run """" & scriptPath & """", 0, False
