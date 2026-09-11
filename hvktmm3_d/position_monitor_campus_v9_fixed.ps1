$ErrorActionPreference = "Stop"

cd D:\hvktmm3_d

Write-Host "============================================================"
Write-Host "SIONNA EVE CHANNEL SERVER - Paths.cfr()"
Write-Host "============================================================"
Write-Host "Dung API Sionna RT paths.cfr(), khong tu tinh H(f) bang tay."
Write-Host "Pi goi: http://172.19.32.199:8765/eve-channel"
Write-Host "Dung server: Ctrl+C"
Write-Host "============================================================"

python .\sionna_eve_server_v3_cfr.py `
  --host 0.0.0.0 `
  --port 8765 `
  --glb .\campus_chinh_chieu_cao.glb `
  --samples 20000 `
  --max-depth 3
