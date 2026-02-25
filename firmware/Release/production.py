import os
import sys
import datetime
import subprocess


def to_little_endian(buf):
    ret = [0] * 12
    ret[0] = buf[3]
    ret[1] = buf[2]
    ret[2] = buf[1]
    ret[3] = buf[0]
    ret[4] = buf[7]
    ret[5] = buf[6]
    ret[6] = buf[5]
    ret[7] = buf[4]
    ret[8] = buf[11]
    ret[9] = buf[10]
    ret[10] = buf[9]
    ret[11] = buf[8]
    return ret


def generate_key(pUID):
    pKey = [0] * 12
    pCustomID = [0x11, 0x07, 0x79, 0x52, 0x87, 0x94, 0x39, 0x86, 0x99, 0x73, 0x80, 0x23]
    pKey[0] = ((pUID[0] & pCustomID[1]) ^ pCustomID[2]) ^ pCustomID[3]
    pKey[1] = (pUID[1] ^ (pCustomID[2] & pCustomID[3])) ^ pCustomID[4]
    pKey[2] = (pUID[2] | (pCustomID[3] ^ pCustomID[4])) | pCustomID[5]
    pKey[3] = (pUID[3] ^ pCustomID[4]) | (pCustomID[5] ^ pCustomID[6])
    pKey[4] = (pCustomID[4] ^ (pUID[5] & pCustomID[6])) ^ pCustomID[7]
    pKey[5] = pCustomID[5] | (pUID[6] ^ (pCustomID[7] & pCustomID[8]))
    pKey[6] = ((pCustomID[6] & pUID[7]) ^ pCustomID[6]) | pCustomID[9]
    pKey[7] = (pCustomID[7] ^ pUID[8]) | (pCustomID[9] ^ pCustomID[10])
    pKey[8] = (pCustomID[8] ^ (pCustomID[9] & pUID[10])) ^ pCustomID[11]
    pKey[9] = (pCustomID[9] ^ pCustomID[10]) | (pUID[11] & pCustomID[0])
    pKey[10] = ((pCustomID[10] & pCustomID[11]) ^ pUID[0]) ^ pCustomID[1]
    pKey[11] = (pCustomID[11] ^ (pCustomID[0] & pUID[1])) ^ pCustomID[2]
    return pKey


if __name__ == "__main__":
    app_name = sys.argv[1]
    boot_name = sys.argv[2]

    # # disable readout protection
    # print("disable readout protection ...")
    # cmd = subprocess.Popen(
    #     "STM32_Programmer_CLI.exe -c port=JLINK -rdu",
    #     shell=True,
    #     stdout=subprocess.PIPE,
    # )

    cmd = subprocess.Popen(
        "STM32_Programmer_CLI.exe -c port=JLINK -e all -r32 0x1FFF7590 12",
        shell=True,
        stdout=subprocess.PIPE,
    )
    r = cmd.stdout.read()
    idx = r.find(b"0x1FFF7590 :")
    if idx == -1:
        print("UID read FAIL")
        exit()
    uid_bytes = r[idx + 13 : idx + 13 + 26]
    uid_bytes = uid_bytes.replace(b" ", b"")
    uid = [int(uid_bytes[i : i + 2], 16) for i in range(0, len(uid_bytes), 2)]

    uid = to_little_endian(uid)

    key = generate_key(uid)
    key = to_little_endian(key)

    key1 = "0x"
    for i in range(4):
        key1 += hex(key[i])[2:].zfill(2).upper()
    key2 = "0x"
    for i in range(4):
        key2 += hex(key[4 + i])[2:].zfill(2).upper()
    key3 = "0x"
    for i in range(4):
        key3 += hex(key[8 + i])[2:].zfill(2).upper()

    # program key
    # program key ""UID_KEY_ADDR_START 0x0801E000 defined in flash.h""
    cmd_str = "-w32 0x0801E000 " + key1 + " " + key2 + " " + key3 + " 0x34492343" + " -v"
    cmd = subprocess.Popen(
        "STM32_Programmer_CLI.exe -c port=JLINK " + cmd_str,
        shell=True,
        stdout=subprocess.PIPE,
    )
    r = cmd.stdout.read()
    idx = r.find(b"Download verified successfully")
    if idx == -1:
        print("write key FAIL")
        exit()

    # program app
    print("program app ...")
    cmd = subprocess.Popen(
        "STM32_Programmer_CLI.exe -c port=JLINK -d " + app_name + " 0x08000000 -v",
        shell=True,
        stdout=subprocess.PIPE,
    )
    r = cmd.stdout.read()
    idx = r.find(b"Download verified successfully")
    if idx == -1:
        print("write app FAIL")
        exit()

    # program boot
    print("program boot ...")
    cmd = subprocess.Popen(
        "STM32_Programmer_CLI.exe -c port=JLINK -d " + boot_name + " 0x0801D000 -v",
        shell=True,
        stdout=subprocess.PIPE,
    )
    r = cmd.stdout.read()
    idx = r.find(b"Download verified successfully")
    if idx == -1:
        print("write boot FAIL")
        exit()

    # program option byte
    print("program option byte ...")
    cmd = subprocess.Popen(
        "STM32_Programmer_CLI.exe -c port=JLINK -ob nSWBOOT0=0 -ob RDP=0xBB",
        shell=True,
        stdout=subprocess.PIPE,
    )

    now = datetime.datetime.now()
    print("Production PASS " + now.strftime("%Y-%m-%d %H:%M:%S"))
