param(
    [string]$PiHost = "172.19.32.193",
    [string]$PiUser = "pi5"
)

$ErrorActionPreference = "Stop"

# ============================================================
# CẤU HÌNH CỐ ĐỊNH
# ============================================================
$BaseDir   = "D:\hvktmm3_d"
$Blender   = "E:\blender\blender.exe"

$Renderer  = Join-Path $BaseDir "render_gps_topdown_iot_visual_v11.py"
$Glb       = Join-Path $BaseDir "campus_chinh_chieu_cao.glb"

$RemoteCsv = "/home/pi5/rBS_AIOT/rbs_lien_ket_dinh_ky_v11.csv"
$LocalCsv  = Join-Path $BaseDir "rbs_lien_ket_dinh_ky_live.csv"

$PreviewDir = Join-Path $BaseDir "preview"

# ============================================================
# KIỂM TRA FILE CẦN THIẾT
# ============================================================
if (!(Test-Path $Blender)) {
    throw "Không tìm thấy Blender: $Blender"
}
if (!(Test-Path $Renderer)) {
    throw "Không tìm thấy renderer: $Renderer"
}
if (!(Test-Path $Glb)) {
    throw "Không tìm thấy GLB: $Glb"
}

# ============================================================
# 1) LẤY CSV MỚI NHẤT TỪ RASPBERRY PI
# ============================================================
Write-Host ""
Write-Host "============================================================"
Write-Host "[1/3] Đang lấy dữ liệu GPS mới nhất từ Raspberry Pi..."
Write-Host "============================================================"

$RemoteSpec = "${PiUser}@${PiHost}:$RemoteCsv"

# Dùng PSCP của PuTTY.
# Nếu pscp đã có trong PATH thì gọi trực tiếp.
# Nếu chưa, thử đường dẫn cài PuTTY mặc định.
$PscpCmd = Get-Command pscp -ErrorAction SilentlyContinue

if ($PscpCmd) {
    & pscp $RemoteSpec $LocalCsv
}
elseif (Test-Path "C:\Program Files\PuTTY\pscp.exe") {
    & "C:\Program Files\PuTTY\pscp.exe" $RemoteSpec $LocalCsv
}
else {
    throw "Không tìm thấy pscp. Hãy cài PuTTY hoặc thêm pscp vào PATH."
}

if ($LASTEXITCODE -ne 0) {
    throw "Không lấy được CSV từ Raspberry Pi."
}

if (!(Test-Path $LocalCsv)) {
    throw "CSV chưa được tải về Windows."
}

Write-Host "[OK] CSV hiện tại: $LocalCsv"

# ============================================================
# 2) RENDER ẢNH DIGITAL TWIN
# ============================================================
Write-Host ""
Write-Host "============================================================"
Write-Host "[2/3] Đang chụp vị trí SU - rBS - DU trên Digital Twin..."
Write-Host "============================================================"

& $Blender `
    --background `
    --python $Renderer `
    -- `
    --glb $Glb `
    --gps-csv $LocalCsv

if ($LASTEXITCODE -ne 0) {
    throw "Blender render thất bại."
}

# ============================================================
# 3) MỞ ẢNH MỚI NHẤT
# ============================================================
Write-Host ""
Write-Host "============================================================"
Write-Host "[3/3] Hoàn tất"
Write-Host "============================================================"

$LatestPng = Get-ChildItem $PreviewDir -Filter "gps_topdown_iot_visual_v11_*.png" |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 1

if ($LatestPng) {
    Write-Host "[ẢNH] $($LatestPng.FullName)"
    Start-Process $LatestPng.FullName
}
else {
    Write-Warning "Render xong nhưng chưa tìm thấy PNG V10 trong $PreviewDir"
}
