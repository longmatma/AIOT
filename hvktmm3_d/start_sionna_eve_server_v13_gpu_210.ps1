$ErrorActionPreference = "Stop"

cd D:\hvktmm3_d

$Py = "D:\hvktmm3_d\.venv_sionna_gpu_210\Scripts\python.exe"

if (!(Test-Path $Py)) {
    throw "Khong tim thay GPU venv: $Py"
}

$env:SIONNA_MITSUBA_VARIANT = "cuda_ad_mono_polarized"

Write-Host "================================================================"
Write-Host "SIONNA EVE - GPU 2.1.0 / CUDA / SCENE ONCE / ADAPTIVE 20K"
Write-Host "================================================================"
Write-Host "Python    : $Py"
Write-Host "GPU       : NVIDIA CUDA via Mitsuba / Dr.Jit"
Write-Host "Variant   : cuda_ad_mono_polarized"
Write-Host "Scene     : load 1 lan khi server start"
Write-Host ""
Write-Host "LIVE      : 2,000 samples / depth 2"
Write-Host "RETRY 1   : 5,000 samples / depth 3"
Write-Host "RETRY 2   : 20,000 samples / depth 4"
Write-Host ""
Write-Host "Pi goi    : http://172.19.32.199:8765/eve-channel"
Write-Host "Dung      : Ctrl+C"
Write-Host "================================================================"

& $Py .\sionna_eve_server_v13_gpu_210_scene_once.py `
  --host 0.0.0.0 `
  --port 8765 `
  --glb .\campus_chinh_chieu_cao.glb `
  --samples 2000 `
  --max-depth 2 `
  --retry-samples-1 5000 `
  --retry-depth-1 3 `
  --retry-samples-2 20000 `
  --retry-depth-2 4
