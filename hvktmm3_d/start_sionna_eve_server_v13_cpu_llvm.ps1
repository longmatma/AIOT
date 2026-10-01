$ErrorActionPreference = "Stop"

Set-Location $PSScriptRoot

$Py = Join-Path $PSScriptRoot ".venv_sionna_gpu_210\Scripts\python.exe"
$LlvmDll = "C:\Program Files\LLVM\bin\LLVM-C.dll"

if (!(Test-Path $Py)) {
    throw "Khong tim thay venv: $Py"
}
if (!(Test-Path $LlvmDll)) {
    throw "Khong tim thay LLVM-C.dll: $LlvmDll"
}

$env:DRJIT_LIBLLVM_PATH = $LlvmDll
$env:SIONNA_MITSUBA_VARIANT = "llvm_ad_mono_polarized"

Write-Host "================================================================"
Write-Host "SIONNA EVE - CPU LLVM 2.1.0 / SCENE ONCE / MULTI-NODE"
Write-Host "================================================================"
Write-Host "Python    : $Py"
Write-Host "Backend   : CPU LLVM via Mitsuba / Dr.Jit"
Write-Host "LLVM DLL  : $LlvmDll"
Write-Host "Variant   : llvm_ad_mono_polarized"
Write-Host "Scene     : load 1 lan khi server start"
Write-Host ""
Write-Host "LIVE      : 2,000 samples / depth 2"
Write-Host "RETRY 1   : 5,000 samples / depth 3"
Write-Host "RETRY 2   : 20,000 samples / depth 4"
Write-Host ""
Write-Host "Pi goi    : http://<IP-WINDOWS>:8765/eve-channel"
Write-Host "Dung      : Ctrl+C"
Write-Host "================================================================"

& $Py .\sionna_eve_server_v13_cpu_llvm_multinode.py `
  --host 0.0.0.0 `
  --port 8765 `
  --glb .\campus_chinh_chieu_cao.glb `
  --samples 2000 `
  --max-depth 2 `
  --retry-samples-1 5000 `
  --retry-depth-1 3 `
  --retry-samples-2 20000 `
  --retry-depth-2 4
