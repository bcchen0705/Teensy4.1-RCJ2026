// Bit 位址選通：把多工器的通道編號 ch 轉成 s0~s3 的控制訊號

digitalWrite(s0, (ch >> 0) & 1); // bit0 → s0，右移 0 位，取最低位
digitalWrite(s1, (ch >> 1) & 1); // bit1 → s1，右移 1 位，取下一位
digitalWrite(s2, (ch >> 2) & 1); // bit2 → s2
digitalWrite(s3, (ch >> 3) & 1); // bit3 → s3，最高位

// 範例：ch = 3 → 二進位 0011
// s0 = 1 (bit0)
// s1 = 1 (bit1)
// s2 = 0 (bit2)
// s3 = 0 (bit3)
// 最後對應 (s3, s2, s1, s0) = 0 0 1 1

// 修正 sensor 位置：30號和31號 sensor 物理裝反，所以交換 index
int mapIndex(int i){
    if(i == 30) return 31; // sensor 30 → map 到 31
    if(i == 31) return 30; // sensor 31 → map 到 30
    return i;              // 其他不變
}

// 用陣列一次定義所有 sensor 的硬體對應
const uint8_t mapTable[32] = {
    0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,
    16,17,18,19,20,21,22,23,24,25,26,27,28,29,31,30
};

// 循環讀 32 顆 sensor
for(int i = 0; i < 32; i++){ 
    ls_data[i] = readMux(mapTable[i], (i < 16) ? M1 : M2); 
    // 三元運算符 (condition ? value_if_true : value_if_false)
    // i < 16 → 讀 M1，多工器1
    // i >=16 → 讀 M2，多工器2
}

