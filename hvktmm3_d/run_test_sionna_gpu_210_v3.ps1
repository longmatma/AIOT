$ErrorActionPreference = "Stop"

cd D:\hvktmm3_d

$Py = "D:\hvktmm3_d\.venv_sionna_gpu_210\Scripts\python.exe"

if (!(Test-Path $Py)) {
    throw "Chua co GPU venv. Hay chay setup_sionna_gpu_210.ps1 truoc."
}

& $Py .\test_sionna_gpu_210_v3.py
