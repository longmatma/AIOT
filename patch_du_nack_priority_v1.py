from pathlib import Path
from datetime import datetime
import shutil

ROOT = Path(__file__).resolve().parent
HMI = ROOT / "DU" / "src" / "hmi_du.cpp"

if not HMI.exists():
    raise SystemExit(f"[FAIL] Khong tim thay: {HMI}")

t = HMI.read_text(encoding="utf-8")

for marker in (
    "static volatile bool cho_phep_nack = false;",
    "static volatile bool yeu_cau_auto_ack = false;",
    "void HMI_DU_Bao_Phat_Xong(uint64_t session_id)",
    "AUTO ACK: tao dung 1 transaction sau khi PLAY xong.",
    "MANUAL NACK: nut GPIO6 active LOW, debounce 25 ms.",
):
    if marker not in t:
        raise SystemExit(
            f"[FAIL] hmi_du.cpp khong dung baseline mong doi: thieu {marker!r}"
        )

if "DU_NACK_WINDOW_MS" in t or "NACK PRIORITY V1" in t:
    raise SystemExit("[FAIL] NACK PRIORITY V1 co ve da duoc patch truoc do.")

stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
backup = ROOT / f"backup_before_du_nack_priority_v1_{stamp}"
backup.mkdir(parents=True, exist_ok=False)

dst = backup / HMI.relative_to(ROOT)
dst.parent.mkdir(parents=True, exist_ok=True)
shutil.copy2(HMI, dst)

print(f"[BACKUP] {backup}")


def rep(text, old, new, label):
    n = text.count(old)
    if n != 1:
        raise RuntimeError(
            f"[STOP] {label}: can dung 1 vi tri, tim thay {n}"
        )
    print(f"[PATCH] {label}")
    return text.replace(old, new, 1)


old = '''static constexpr uint32_t DU_PACKET_PULSE_MS = 70UL;
static constexpr uint32_t DU_DATA_GAP_TIMEOUT_MS = 300UL;
'''

new = '''static constexpr uint32_t DU_PACKET_PULSE_MS = 70UL;
static constexpr uint32_t DU_DATA_GAP_TIMEOUT_MS = 300UL;

// ============================================================
// NACK PRIORITY V1
//
// Sau khi DU phat xong audio:
//   - mo cua so NACK trong 1200 ms;
//   - neu GPIO6 duoc bam -> gui NACK NGAY;
//   - neu khong bam -> het 1200 ms moi AUTO_ACK.
//
// Day la decision window SAU playback, nen KHONG lam cham
// PTT_RELEASE -> DU_PLAY va KHONG chen vao VOICE/FEC.
// ============================================================
static constexpr uint32_t DU_NACK_WINDOW_MS = 1200UL;
'''

t = rep(t, old, new, "add 1200ms NACK decision window")


old = '''static volatile bool cho_phep_nack = false;
static volatile bool yeu_cau_auto_ack = false;
static volatile uint64_t phien_auto_ack = 0;
static volatile uint64_t phien_vua_phat_xong = 0;
'''

new = '''static volatile bool cho_phep_nack = false;
static volatile bool yeu_cau_auto_ack = false;
static volatile uint64_t phien_auto_ack = 0;
static volatile uint64_t phien_vua_phat_xong = 0;

// Moc het cua so cho nguoi dung bam NACK.
// =0 khi khong co decision window.
static volatile uint32_t moc_het_cua_so_nack_ms = 0;
'''

t = rep(t, old, new, "add NACK window state")


old = '''    cho_phep_nack = false;
    yeu_cau_auto_ack = false;
    phien_auto_ack = 0;
    phien_vua_phat_xong = 0;

    dang_cho_confirm = false;
'''

new = '''    cho_phep_nack = false;
    yeu_cau_auto_ack = false;
    phien_auto_ack = 0;
    phien_vua_phat_xong = 0;
    moc_het_cua_so_nack_ms = 0;

    dang_cho_confirm = false;
'''

t = rep(t, old, new, "reset NACK window on new session")


old = '''void HMI_DU_Bao_Phat_Xong(uint64_t session_id)
{
    if (session_id == 0)
        return;

    portENTER_CRITICAL(&hmi_du_mux);
    phien_vua_phat_xong = session_id;
    cho_phep_nack = true;
    phien_auto_ack = session_id;
    yeu_cau_auto_ack = true;
    khoa_beacon_den_ms = millis() + 1500UL;
    portEXIT_CRITICAL(&hmi_du_mux);

    Serial.printf(
        "[DU HMI] AUTO_ACK REQUEST | SESSION=%016llX | GPIO6=NACK neu nghe khong ro\\n",
        (unsigned long long)session_id
    );
}
'''

new = '''void HMI_DU_Bao_Phat_Xong(uint64_t session_id)
{
    if (session_id == 0)
        return;

    const uint32_t now = millis();

    portENTER_CRITICAL(&hmi_du_mux);
    phien_vua_phat_xong = session_id;

    // Cho phep NACK ngay khi playback ket thuc.
    cho_phep_nack = true;

    // AUTO_ACK chi duoc phep chay SAU cua so NACK.
    phien_auto_ack = session_id;
    yeu_cau_auto_ack = true;
    moc_het_cua_so_nack_ms = now + DU_NACK_WINDOW_MS;

    // Trong cua so decision, khong cho telemetry/beacon chen vao.
    khoa_beacon_den_ms = now + DU_NACK_WINDOW_MS + 300UL;
    portEXIT_CRITICAL(&hmi_du_mux);

    Serial.printf(
        "[DU HMI] NACK WINDOW %lu ms | SESSION=%016llX | "
        "GPIO6=NACK | KHONG BAM -> AUTO_ACK\\n",
        (unsigned long)DU_NACK_WINDOW_MS,
        (unsigned long long)session_id
    );
}
'''

t = rep(t, old, new, "delay AUTO_ACK until NACK window expires")


old = '''        moc_tat_led_confirm_ms = millis() + 2000UL;
        cho_phep_nack = true;
        khoa_beacon_den_ms = millis() + 1000UL;
        khop = true;
'''

new = '''        moc_tat_led_confirm_ms = millis() + 2000UL;

        // Transaction da duoc SU confirm -> khong cho late-NACK.
        cho_phep_nack = false;
        yeu_cau_auto_ack = false;
        moc_het_cua_so_nack_ms = 0;

        khoa_beacon_den_ms = millis() + 1000UL;
        khop = true;
'''

t = rep(t, old, new, "close NACK after SU confirm")


old = '''        // ----------------------------------------------------
        // AUTO ACK: tao dung 1 transaction sau khi PLAY xong.
        // ----------------------------------------------------
        portENTER_CRITICAL(&hmi_du_mux);
        if (yeu_cau_auto_ack && !dang_cho_confirm)
        {
            code_can_gui = USER_RESPONSE_ACK;
            session_can_gui = phien_auto_ack;
            yeu_cau_auto_ack = false;
        }
        portEXIT_CRITICAL(&hmi_du_mux);

        // ----------------------------------------------------
        // MANUAL NACK: nut GPIO6 active LOW, debounce 25 ms.
        // ----------------------------------------------------
        bool nack_hien_tai = digitalRead(CHAN_NUT_NACK_DU);
        bool co_the_nack;
        bool pending;
        uint64_t session_vua_phat;

        portENTER_CRITICAL(&hmi_du_mux);
        co_the_nack = cho_phep_nack;
        pending = dang_cho_confirm;
        session_vua_phat = phien_vua_phat_xong;
        portEXIT_CRITICAL(&hmi_du_mux);

        if (
            code_can_gui == 0
            && co_the_nack
            && !pending
            && nack_truoc == HIGH
            && nack_hien_tai == LOW
        )
        {
            vTaskDelay(pdMS_TO_TICKS(25));

            if (digitalRead(CHAN_NUT_NACK_DU) == LOW)
            {
                code_can_gui = USER_RESPONSE_NACK;
                session_can_gui = session_vua_phat;
            }
        }

        nack_truoc = nack_hien_tai;
'''

new = '''        // ----------------------------------------------------
        // NACK PRIORITY V1
        //
        // MANUAL NACK duoc kiem tra TRUOC AUTO_ACK.
        // Nut active LOW, debounce 25 ms.
        //
        // Neu nguoi dung da bam/giu nut ngay luc playback vua ket
        // thuc thi van nhan NACK; khong bat buoc canh HIGH->LOW.
        // ----------------------------------------------------
        bool nack_hien_tai = digitalRead(CHAN_NUT_NACK_DU);
        bool co_the_nack;
        bool pending;
        uint64_t session_vua_phat;

        portENTER_CRITICAL(&hmi_du_mux);
        co_the_nack = cho_phep_nack;
        pending = dang_cho_confirm;
        session_vua_phat = phien_vua_phat_xong;
        portEXIT_CRITICAL(&hmi_du_mux);

        if (
            co_the_nack
            && !pending
            && nack_hien_tai == LOW
        )
        {
            vTaskDelay(pdMS_TO_TICKS(25));

            if (digitalRead(CHAN_NUT_NACK_DU) == LOW)
            {
                code_can_gui = USER_RESPONSE_NACK;
                session_can_gui = session_vua_phat;

                // NACK da thang -> huy AUTO_ACK cua cung session.
                portENTER_CRITICAL(&hmi_du_mux);
                cho_phep_nack = false;
                yeu_cau_auto_ack = false;
                moc_het_cua_so_nack_ms = 0;
                portEXIT_CRITICAL(&hmi_du_mux);

                Serial.printf(
                    "[DU HMI] GPIO6 NACK DUOC CHON | SESSION=%016llX\\n",
                    (unsigned long long)session_can_gui
                );
            }
        }

        nack_truoc = nack_hien_tai;

        // ----------------------------------------------------
        // AUTO ACK:
        // Chi tao transaction neu:
        //   1) nguoi dung KHONG bam NACK;
        //   2) cua so 1200 ms da het;
        //   3) chua co transaction dang cho confirm.
        // ----------------------------------------------------
        if (code_can_gui == 0)
        {
            const uint32_t now = millis();

            portENTER_CRITICAL(&hmi_du_mux);

            const bool cua_so_da_het =
                moc_het_cua_so_nack_ms != 0
                &&
                (int32_t)(now - moc_het_cua_so_nack_ms) >= 0;

            if (
                yeu_cau_auto_ack
                &&
                !dang_cho_confirm
                &&
                cua_so_da_het
            )
            {
                code_can_gui = USER_RESPONSE_ACK;
                session_can_gui = phien_auto_ack;

                yeu_cau_auto_ack = false;
                cho_phep_nack = false;
                moc_het_cua_so_nack_ms = 0;
            }

            portEXIT_CRITICAL(&hmi_du_mux);

            if (code_can_gui == USER_RESPONSE_ACK)
            {
                Serial.printf(
                    "[DU HMI] NACK WINDOW HET -> AUTO_ACK | SESSION=%016llX\\n",
                    (unsigned long long)session_can_gui
                );
            }
        }
'''

t = rep(t, old, new, "manual NACK priority before delayed AUTO_ACK")


old = '''                dang_cho_confirm = false;
                da_nhan_confirm = false;
                ma_phan_hoi_dang_cho = 0;
                phien_phan_hoi_dang_cho = 0;
                moc_tat_led_confirm_ms = 0;
                portEXIT_CRITICAL(&hmi_du_mux);
'''

new = '''                dang_cho_confirm = false;
                da_nhan_confirm = false;
                ma_phan_hoi_dang_cho = 0;
                phien_phan_hoi_dang_cho = 0;
                moc_tat_led_confirm_ms = 0;

                cho_phep_nack = false;
                yeu_cau_auto_ack = false;
                moc_het_cua_so_nack_ms = 0;

                portEXIT_CRITICAL(&hmi_du_mux);
'''

t = rep(t, old, new, "close NACK/AUTO state after confirm timeout")


HMI.write_text(t, encoding="utf-8")

final = HMI.read_text(encoding="utf-8")

required = (
    "DU_NACK_WINDOW_MS = 1200UL",
    "[DU HMI] NACK WINDOW %lu ms",
    "[DU HMI] GPIO6 NACK DUOC CHON",
    "[DU HMI] NACK WINDOW HET -> AUTO_ACK",
)

for marker in required:
    if marker not in final:
        raise RuntimeError(f"[STOP] Sanity fail: thieu {marker!r}")

if "AUTO ACK: tao dung 1 transaction sau khi PLAY xong." in final:
    raise RuntimeError("[STOP] Block AUTO_ACK cu van con.")

print()
print("[DONE] DU NACK PRIORITY V1")
print("[WINDOW] 1200 ms sau playback de nguoi dung bam GPIO6=NACK")
print("[NACK] bam trong window -> gui NACK ngay, huy AUTO_ACK")
print("[AUTO ACK] chi gui sau 1200 ms neu khong co NACK")
print("[SU LED] khong can sua: SU da co mau LED NACK nhanh/cham")
print("[LATENCY] KHONG anh huong PTT_RELEASE -> DU_PLAY; window nam SAU playback")
print("[KEEP] AES/FEC/VOICE/session/telemetry/rBS/STM32 khong sua")
print("[NEXT] Build DU. Neu SUCCESS thi upload CHI DU va test ACK/NACK.")
