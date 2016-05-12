@cd %1\Code
@"C:\Program Files (x86)\Microsoft Visual Studio 14.0\Common7\IDE\devenv.exe" "%1.sln" /build Release
::@%WINDIR%\Microsoft.NET\Framework\v4.0.30319\MSBuild.exe %1.sln /build Release
@cd ..
@cd ..