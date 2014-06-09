
Idx:{//ISP_NVRAM_REG_INDEX_STRUCT
    Shading     :0,
    OB          :0,
    DM          :0,
    DP          :0,
    NR1         :2,
    NR2         :2,
    EE          :2,
    Saturation  :2,
    Contrast    :4,
    Hue         :1,
    CCM         :1,
    Gamma       :0
},
Shading:{
    {set:{//00 Preview
        0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x10000000,0x90409030,0xC2019900,0x00420033,0x20202020,
    }},
    {set:{//01 Capture
        0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x10000000,0xF050F03C,0x00000000,0x0052003F,0x20202020,
    }},
    {set:{//02
        0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,
    }}
},
OB:{
    {set:{//00
        0xA9A9A9A9
    }}
        },
DM:{
    {set:{//00 Preview ISO100/ISO200/ISO400
        0x00000100, 0x0A800810, 0x0020083F, 
    }},
    {set:{//01 Preview ISO800
        0x00000100, 0x0A800810, 0x0020083F, 
    }},
    {set:{//02 Preview ISO1600
        0x00000100, 0x0A800810, 0x0020083F, 
    }},
    {set:{//03 Capture ISO100/ISO200
        0x00000100, 0x0A800810, 0x0020083F, 
    }},
    {set:{//04 Capture ISO400
        0x00000100, 0x0A800810, 0x0020083F, 
    }},
    {set:{//05 Capture ISO800
        0x00000100, 0x0A800810, 0x0020083F, 
    }},
    {set:{//06 Capture ISO1600
        0x00000100, 0x0A800810, 0x0020083F, 
    }}
            },
DP:{// Auto Defect
    {set:{//00
        0x000006E6, 0x50285050, 0x006003A0, 0x00300050, 0x000009B7, 
    }},
    {set:{//01
        0x000006E7, 0x50285050, 0x006003A0, 0x00300050, 0x000009B7, 
    }},
    {set:{//02
        0x000006E7, 0x50285050, 0x006003A0, 0x00300050, 0x000009B7, 
    }},
    {set:{//03 Disable (Do not modify me)
        0x00000000, 0x50285050, 0x006003A0, 0x00300050, 0x000008B7, 
    }}
        },
NR1:{
    {set:{//00 Preview ISO100
        0x000002E7, 0x00001140, 0x092328C8, 0x000008AC, 0x05050507, 0x090B0C0C, 0x05050507, 
        0x090B0C0C, 0x05050507, 0x090B0C0C, 0x05050507, 0x090B0C0C, 0x0206090D, 0x10101010, 
    }},
    {set:{//01 Preview ISO200
        0x000002E1, 0x00001140, 0x092328C8, 0x000008AC, 0x05050507, 0x090B0C0C, 0x05050507, 
        0x090B0C0C, 0x05050507, 0x090B0C0C, 0x05050507, 0x090B0C0C, 0x0206090D, 0x10101010, 
    }},
    {set:{//02 Preview ISO400
        0x000002E1, 0x00001140, 0x092528C8, 0x000008AC, 0x05050507, 0x090B0C0C, 0x05050507, 
        0x090B0C0C, 0x05050507, 0x090B0C0C, 0x05050507, 0x090B0C0C, 0x0206090D, 0x10101010, 
    }},
    {set:{//03 Preview ISO800
        0x000002E7, 0x00001140, 0x092C28C8, 0x000008AC, 0x05050507, 0x090B0C0C, 0x05050507, 
        0x090B0C0C, 0x05050507, 0x090B0C0C, 0x05050507, 0x090B0C0C, 0x0206090D, 0x10101010, 
    }},
    {set:{//04 Preview ISO1600
        0x000002E7, 0x00001140, 0x092D28C8, 0x000008AC, 0x05050507, 0x090B0C0C, 0x05050507, 
        0x090B0C0C, 0x05050507, 0x090B0C0C, 0x05050507, 0x090B0C0C, 0x0206090D, 0x10101010, 
    }},
    {set:{//05 Capture ISO100
        0x000002E7, 0x00001140, 0x092328C8, 0x000008AC, 0x0A0A0505, 0x06080808, 0x0A0A0505, 
        0x06080808, 0x0A0A0505, 0x06080808, 0x0A0A0505, 0x06080808, 0x0206090D, 0x10101010, 
    }},
    {set:{//06 Capture ISO200
        0x000002E7, 0x00001140, 0x092328C8, 0x000008AC, 0x0A0A0505, 0x06080808, 0x0A0A0505, 
        0x06080808, 0x0A0A0505, 0x06080808, 0x0A0A0505, 0x06080808, 0x0206090D, 0x10101010, 
    }},
    {set:{//07 Capture ISO400
        0x000002E7, 0x00001140, 0x092528C8, 0x000008AC, 0x05050507, 0x090B0C0C, 0x05050507, 
        0x090B0C0C, 0x05050507, 0x090B0C0C, 0x05050507, 0x090B0C0C, 0x0206090D, 0x10101010, 
    }},
    {set:{//08 Capture ISO800
        0x000002E7, 0x00001140, 0x092C28C8, 0x000008AC, 0x0A0A0A0E, 0x12161818, 0x0A0A0A0E, 
        0x12161818, 0x0A0A0A0E, 0x12161818, 0x0A0A0A0E, 0x12161818, 0x0206090D, 0x10101010, 
    }},
    {set:{//09 Capture IS1600
        0x000002E7, 0x00001140, 0x092D28C8, 0x000008AC, 0x0A0A0A0E, 0x12161818, 0x0A0A0A0E, 
        0x12161818, 0x0A0A0A0E, 0x12161818, 0x0A0A0A0E, 0x12161818, 0x0206090D, 0x10101010, 
    }},
    {set:{//10 Disable (Do not modify me)
        0x000000C0, 0x000011A0, 0x094428A0, 0x000007AF, 0x03050709, 0x0B0D0F11, 0x03050709, 
        0x0B0D0F11, 0x03050709, 0x0B0D0F11, 0x03050709, 0x0B0D0F11, 0x0406090D, 0x10101010, 
    }}
        },
NR2:{
    {set:{//00 Preview ISO100
        0x00200003, 0x00191414, 0x00D27788, 0x20406090, 0x305888C8, 0x021072CA, 0x08080808, 0x10101010, 0x01AF5B48, 0x0000056B, 0x0305080C, 
    }},
    {set:{//01 Preview ISO200
        0x00200000, 0x00191414, 0x00D27788, 0x306090D0, 0x4080C0F0, 0x021072CA, 0x08080808, 0x10101010, 0x01AF5B48, 0x0000056B, 0x04080C11, 
    }},
    {set:{//02 Preview ISO400
        0x00200000, 0x00191414, 0x00D27768, 0xF0F0F0F0, 0xF0F0F0F0, 0x01307A8E, 0x08080808, 0x10101010, 0x01AF5B48, 0x0000056B, 0x070D131B, 
    }},
    {set:{//03 Preview ISO800
        0x00200003, 0x00191414, 0x00D27788, 0xF0F0F0FC, 0xF0F4F4F4, 0x01707A8E, 0x08080808, 0x10101010, 0x01AF5B48, 0x0000056B, 0x080F151F, 
    }},
    {set:{//04 Preview ISO1600
        0x00200003, 0x00191414, 0x00D27788, 0xF0F0F0FC, 0xF0F4FCFC, 0x01707A8E, 0x08080808, 0x10101010, 0x01AF5B48, 0x0000056B, 0x080F151F, 
    }},
    {set:{//05 Capture Mode0 ISO100
        0x00200003, 0x00191414, 0x00D27788, 0x20406090, 0x305888C8, 0x021072CA, 0x08080808, 0x10101010, 0x01AF5B48, 0x0000056B, 0x0305080C, 
    }},
    {set:{//06 Capture Mode0 ISO200
        0x00200003, 0x00191414, 0x00D27788, 0x306090D0, 0x4080C0F0, 0x021072CA, 0x08080808, 0x10101010, 0x01AF5B48, 0x0000056B, 0x04080C11, 
    }},
    {set:{//07 Capture Mode0 ISO400
        0x00200003, 0x00191414, 0x00D27788, 0xF0F0F0F0, 0xF0F0F0F0, 0x01307A8E, 0x08080808, 0x10101010, 0x01AF5B48, 0x0000056B, 0x070D131B, 
    }},
    {set:{//08 Capture Mode0 ISO800
        0x00200003, 0x00191414, 0x00D27788, 0xF0F0F0FC, 0xF0F4F4F4, 0x01707A8E, 0x08080808, 0x10101010, 0x01AF5B48, 0x0000056B, 0x080F151F, 
    }},
    {set:{//09 Capture Mode0 ISO1600
        0x00200003, 0x00191414, 0x00D27788, 0xF0F0F0FC, 0xF0F4FCFC, 0x01707A8E, 0x08080808, 0x10101010, 0x01AF5B48, 0x0000056B, 0x080F151F, 
    }},
    {set:{//10 Capture Mode1 ISO100
        0x00210003, 0x00191414, 0x00D27788, 0x20406090, 0x305888C8, 0x021072CA, 0x08080808, 0x10101010, 0x01AF5B48, 0x0000056B, 0x0305080C, 
    }},
    {set:{//11 Capture Mode1 ISO200
        0x00210003, 0x00191414, 0x00D27788, 0x306090D0, 0x4080C0F0, 0x021072CA, 0x08080808, 0x10101010, 0x01AF5B48, 0x0000056B, 0x04080C11, 
    }},
    {set:{//12 Capture Mode1 ISO400
        0x00210003, 0x00191414, 0x00D27788, 0xF0F0F0F0, 0xF0F0F0F0, 0x01307A8E, 0x08080808, 0x10101010, 0x01AF5B48, 0x0000056B, 0x070D131B, 
    }},
    {set:{//13 Capture Mode1 ISO800
        0x00210003, 0x00191414, 0x00D27788, 0xF0F0F0FC, 0xF0F4F4F4, 0x01707A8E, 0x08080808, 0x10101010, 0x01AF5B48, 0x0000056B, 0x080F151F, 
    }},
    {set:{//14 Capture Mode1 ISO1600
        0x00210003, 0x00191414, 0x00D27788, 0xF0F0F0FC, 0xF0F4FCFC, 0x01707A8E, 0x08080808, 0x10101010, 0x01AF5B48, 0x0000056B, 0x080F151F, 
    }},
    {set:{//15 Disable (Do not modify me)
        0x00000000, 0x0001FF01, 0x00523264, 0x04090B0F, 0x050A0B10, 0x02107294, 0x08101820, 0x10080604, 0x01AF5B43, 0x0000056B, 0x0306070A, 
    }}
},
EE:{// EE
    {set:{//00 Preview ISO100 (middle sharpness)
        0x0000002C, 0x0037372D, 0x0000000D, 0x0244341C, 0x03C00182, 0x033100AA, 0xFFFFFD0D, 
    }},
    {set:{//01 Preview ISO200 (middle sharpness)
        0x0000002B, 0x0037372C, 0x0000000F, 0x021F311E, 0x03B60282, 0x035C00AA, 0xFFFFFD0D, 
    }},
    {set:{//02 Preview ISO400 (middle sharpness)
        0x00000028, 0x0037372C, 0x0000000F, 0x02E72921, 0x039801A3, 0x034800CB, 0xFFFFFD0D, 
    }},
    {set:{//03 Preview ISO800 (middle sharpness)
        0x00000026, 0x0037372D, 0x000F0063, 0x00ED0495, 0x013401BD, 0x015200E5, 0xFFFFFB0D, 
    }},
    {set:{//04 Preview ISO1600 (middle sharpness)
        0x00000026, 0x007F3721, 0x000E0068, 0x00DB049A, 0x011C01C2, 0x013800EA, 0xFFFFFB0D, 
    }},
    {set:{//05 Capture ISO100 (middle sharpness)
        0x0000002A, 0x0037372D, 0x00320128, 0x01D80A5A, 0x02E70482, 0x033101AA, 0xFFFFFD0D, 
    }},
    {set:{//06 Capture ISO200 (middle sharpness)
        0x0000002A, 0x0037372D, 0x00320128, 0x01D80A5A, 0x02E70482, 0x033101AA, 0xFFFFFD0D, 
    }},
    {set:{//07 Capture ISO400 (middle sharpness)
        0x00000028, 0x0037372D, 0x00120049, 0x0145067B, 0x01A602A3, 0x01D001CB, 0xFFFFFD0D, 
    }},
    {set:{//08 Capture ISO800 (middle sharpness)
        0x00000026, 0x0037372D, 0x000F0063, 0x00ED0495, 0x013401BD, 0x015200E5, 0xFFFFFB0D, 
    }},
    {set:{//09 Capture ISO1600 (middle sharpness)
        0x00000026, 0x007F3721, 0x000E0068, 0x00DB049A, 0x011C01C2, 0x013800EA, 0xFFFFFB0D, 
    }},
    {set:{//10  no one uses this, this is Min EE (low sharpness)
        0x00000024, 0x0037372D, 0x00050023, 0x012C113C, 0x020001D0, 0x020000E7, 0xFFFF0005, 
    }},
    {set:{//11 no one uses this, this is Max EE (high sharpness)
        0x0000002F, 0x00373721, 0x000A0023, 0x03203937, 0x03FF01D0, 0x03FF00E7, 0xFFFF0005, 
    }}
            },
Saturation:{
    {set:{//00 (middle saturation)
        0x00010709, 0x1020E0F0, 0x20455045, 0x20000000, 0xFF00FF00, 0x00000000, 0x00000000, 0x001E140A, 
    }},
    {set:{//01 (middle saturation)
        0x00010708, 0x1020E0F0, 0x20455045, 0x20000000, 0xFF00FF00, 0x00000000, 0x00000000, 0x001E140A, 
    }},
    {set:{//02 (middle saturation)
        0x00010708, 0x1020E0F0, 0x20404540, 0x20000000, 0xFF00FF00, 0x00000000, 0x00000000, 0x001E140A, 
    }},
    {set:{//03 (middle saturation)
        0x00010709, 0x1020E0F0, 0x1C354035, 0x1C000000, 0xFF00FF00, 0x00000000, 0x00000000, 0x001E140A, 
    }},
    {set:{//04 (middle saturation)
        0x00010709, 0x2850A0FF, 0x1A303530, 0x1A000000, 0xFF00FF00, 0x40302000, 0x00000000, 0x00783C1E, 
    }},
    {set:{//05 no one uses this, this is Min Sat. (low saturation)
        0x00010709, 0x2850A0FF, 0x1A303230, 0x1A000000, 0xFF00FF00, 0x40302000, 0x00000000, 0x00783C1E, 
    }},
    {set:{//06 no one uses this, this is Max Sat. (high saturation)
        0x00010709, 0x1020E0F0, 0x20636863, 0x20000000, 0xFF00FF00, 0x00000000, 0x00000000, 0x001E140A, 
    }}
            },
Contrast:{
    //..........................................................................
    // low brightness
    {set:{//00 //  low contrast
        0x00000008, 0x00F00000, 0xFF00003B, 
    }},
    {set:{//01 //  middle contrast
        0x00000008, 0x00F00000, 0xFF000040, 
    }},
    {set:{//02 //  high contrast
        0x00000008, 0x00F00000, 0xFF000045, 
    }},
    //..........................................................................
    // middle brightness
    {set:{//03 //  low contrast
        0x00000008, 0x00000000, 0xFF00003B, 
    }},
    {set:{//04 //  middle contrast
        0x00000000, 0x00000000, 0xFF000040, 
    }},
    {set:{//05 //  high contrast
        0x00000008, 0x00000000, 0xFF000045, 
    }},
    //..........................................................................
    // high brightness
    {set:{//06 //  low contrast
        0x00000008, 0x000A0000, 0xFF00003B, 
    }},
    {set:{//07 //  middle contrast
        0x00000008, 0x000A0000, 0xFF000040, 
    }},
    {set:{//08 //  high contrast
        0x00000008, 0x000A0000, 0xFF000045, 
    }}
            },
Hue:{
    {set:{//00 // low hue
        0x00000002, 0x808062AE, 
    }},
    {set:{//01 // middle hue
        0x00000000, 0x00007F01, 
    }},
    {set:{//02 // high hue
        0x00000002, 0x80806252, 
    }}
},
CCM:{
    {set:{//00
        0x017D0471, 0x040C0449, 0x010B003E, 0x041F0461, 0x01800000, 
    }},
    {set:{//01
        0x017D0471, 0x040C0449, 0x010B003E, 0x041F0461, 0x01800000, 
    }},
    {set:{//02
        0x017D0471, 0x040C0449, 0x010B003E, 0x041F0461, 0x01800000, 
    }}
            },
Gamma:{
    {set:{//00
        0x45231106, 0x9082725E, 0xC5B7A69B, 0xEBE2DAD1, 0xFEFEFBF6, 
    }},
            }
