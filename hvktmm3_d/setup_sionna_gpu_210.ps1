$ErrorActionPreference = "Stop"

cd D:\hvktmm3_d

Write-Host "============================================================"
Write-Host "TAO MOI TRUONG SIONNA RT 2.1.0 RIENG CHO GPU"
Write-Host "============================================================"
Write-Host "Khong sua / khong go package cua Python hien tai."
Write-Host ""

$Venv = "D:\hvktmm3_d\.venv_sionna_gpu_210"
$Py = "$Venv\Scripts\python.exe"

if (Test-Path $Venv) {
    Write-Host "[INFO] Venv da ton tai: $Venv"
} else {
    Write-Host "[1/4] Tao venv bang Python 3.10..."
    py -3.10 -m venv $Venv
}

Write-Host "[2/4] Nang pip/setuptools/wheel..."
& $Py -m pip install --upgrade pip setuptools wheel

Write-Host "[3/4] Cai sionna-rt 2.1.0 va dependency tuong ung..."
& $Py -m pip install --upgrade --no-cache-dir "sionna-rt==2.1.0"

Write-Host "[4/4] In version thuc te..."
& $Py -c "import importlib.metadata as m; print('sionna-rt =',m.version('sionna-rt')); print('mitsuba =',m.version('mitsuba')); print('drjit =',m.version('drjit'))"

Write-Host ""
Write-Host "============================================================"
Write-Host "SETUP XONG"
Write-Host "Python GPU env:"
Write-Host $Py
Write-Host "============================================================"
