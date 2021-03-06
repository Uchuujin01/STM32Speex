const uint16_t power_frames = 196;
const unsigned char power[] = {
    0x1c, 0xc9, 0xd9, 0x80, 0xe8, 0x7d, 0x74, 0xf9, 0xda, 0xff, 0x2f, 0xa6, 
    0x47, 0x6b, 0x8a, 0x94, 0x08, 0x07, 0x39, 0xce, 0x1f, 0x99, 0x02, 0x33, 
    0xa0, 0xb4, 0x6e, 0x70, 0xc0, 0x1c, 0xe7, 0x38, 0x18, 0x0e, 0xc6, 0xee, 
    0x7a, 0x16, 0x79, 0x0d, 0x1d, 0xcc, 0xd0, 0x94, 0x4d, 0x48, 0x0a, 0x13, 
    0x56, 0xa0, 0xb9, 0x68, 0x9c, 0xe3, 0xaa, 0xe7, 0x1b, 0xb9, 0x10, 0xbe, 
    0x19, 0xcd, 0x47, 0x16, 0xf0, 0x39, 0xce, 0x71, 0xb8, 0x5c, 0x00, 0x00, 
    0xb2, 0x79, 0x92, 0x52, 0xd5, 0xbb, 0x5a, 0xea, 0x19, 0xd4, 0x67, 0x24, 
    0xbb, 0xdb, 0x19, 0xc5, 0x77, 0x63, 0x6b, 0xee, 0xc7, 0xb9, 0x62, 0x92, 
    0x28, 0xf0, 0xfb, 0x4f, 0x19, 0xd8, 0x67, 0x20, 0x78, 0xbf, 0xcf, 0x59, 
    0xf6, 0x7f, 0xff, 0xe0, 0x46, 0xea, 0x16, 0x09, 0xbe, 0x18, 0x90, 0xf7, 
    0x19, 0xd0, 0x07, 0x2f, 0xf6, 0xa1, 0x27, 0xb2, 0x8f, 0x67, 0xe6, 0x58, 
    0x17, 0x9f, 0x28, 0xcf, 0xf6, 0x35, 0xf5, 0x9e, 0x19, 0xd4, 0x07, 0x2e, 
    0x91, 0xc0, 0x59, 0xfc, 0x6d, 0xd2, 0xa0, 0x78, 0x79, 0x33, 0xf7, 0x16, 
    0x10, 0xff, 0xd7, 0xf6, 0x19, 0xd9, 0xa7, 0x24, 0xbb, 0xa4, 0xf2, 0xfc, 
    0xed, 0x57, 0xa5, 0x73, 0x20, 0x35, 0x7a, 0x71, 0xd8, 0x71, 0x6e, 0x5c, 
    0x19, 0xd9, 0x87, 0x23, 0xe8, 0x8c, 0x98, 0x07, 0xde, 0x55, 0xf5, 0xa9, 
    0xa2, 0x65, 0x22, 0x57, 0xb7, 0x94, 0xc4, 0x07, 0x19, 0xd8, 0x67, 0x24, 
    0xdb, 0xff, 0x0b, 0x48, 0x49, 0x63, 0x4d, 0x1c, 0x5e, 0x3f, 0x3a, 0x9e, 
    0x8c, 0xbf, 0xc5, 0xec, 0x19, 0xd4, 0x67, 0x27, 0xde, 0xdb, 0x22, 0xb9, 
    0xd3, 0x57, 0x8a, 0xbd, 0x4a, 0x31, 0x25, 0xce, 0xd6, 0x6e, 0x90, 0x97, 
    0x19, 0xd0, 0x57, 0x21, 0xf3, 0xe6, 0xe4, 0xc5, 0x96, 0xad, 0x65, 0xa3, 
    0xe3, 0x64, 0x5f, 0xaa, 0xe6, 0x5b, 0x33, 0x38, 0x19, 0xd4, 0x27, 0x12, 
    0xa7, 0xb4, 0x3f, 0xcc, 0xa3, 0xe3, 0x03, 0x62, 0x31, 0xe7, 0x55, 0x78, 
    0x79, 0x7b, 0x1a, 0x38, 0x19, 0xc5, 0x63, 0x2b, 0x13, 0xc5, 0xf7, 0xcd, 
    0xe7, 0x6b, 0x69, 0x57, 0xe7, 0xb0, 0x0d, 0xce, 0x0a, 0x7b, 0x68, 0xb5, 
    0x1d, 0xc0, 0x05, 0x12, 0xed, 0xa5, 0x51, 0x7e, 0xb5, 0xd8, 0x16, 0xff, 
    0xde, 0xea, 0x42, 0x14, 0x48, 0xb2, 0x56, 0x65, 0x1d, 0xd9, 0x9b, 0x6a, 
    0xbc, 0x09, 0xc7, 0x3e, 0xbe, 0x00, 0x70, 0x01, 0x04, 0xab, 0xa0, 0xb1, 
    0x06, 0x58, 0xad, 0xf1, 0x1f, 0xc1, 0xa1, 0x98, 0xbb, 0x7e, 0x5c, 0x33, 
    0x5a, 0xdf, 0x8b, 0xa3, 0x74, 0xa8, 0x9d, 0x1d, 0x6a, 0x1c, 0xc6, 0x2a, 
    0x1f, 0xc0, 0xa5, 0xe7, 0x10, 0x2a, 0x47, 0x4a, 0x3e, 0x34, 0xbd, 0xcc, 
    0xbc, 0x2c, 0xc1, 0xc2, 0xdb, 0x77, 0xf9, 0x4c, 0x18, 0xd8, 0x55, 0xf6, 
    0xb4, 0x82, 0xa7, 0x4b, 0x2d, 0xef, 0xff, 0xbd, 0x95, 0xb2, 0x83, 0x00, 
    0xc3, 0xb7, 0xb8, 0xfa, 0x1f, 0xce, 0xd1, 0xf6, 0x07, 0xb7, 0x3c, 0x92, 
    0xf4, 0x75, 0x33, 0x25, 0x6e, 0xb6, 0x7c, 0x54, 0xb5, 0xf1, 0x35, 0x5f, 
    0x19, 0x87, 0x51, 0xfd, 0x06, 0xa5, 0x42, 0xd2, 0x9f, 0xd4, 0x4a, 0xd5, 
    0x39, 0xe8, 0x42, 0x2e, 0x9e, 0xf0, 0x09, 0x1b, 0x1b, 0xd0, 0x91, 0xe5, 
    0x0e, 0xaf, 0x32, 0x9a, 0x97, 0xfd, 0xaa, 0x9d, 0x51, 0xf6, 0x6a, 0xe6, 
    0xa4, 0xe5, 0x99, 0x2f, 0x1b, 0xd1, 0x11, 0xd5, 0x07, 0xd3, 0xb9, 0x8a, 
    0x6b, 0xe7, 0x41, 0xc9, 0x39, 0xec, 0x7e, 0xb7, 0x7a, 0xf0, 0x68, 0xb9, 
    0x1b, 0xd1, 0x29, 0xd4, 0x9b, 0xd5, 0x57, 0xca, 0x47, 0x56, 0xbc, 0xd6, 
    0xd5, 0x64, 0xd5, 0x78, 0x8e, 0xfc, 0x09, 0x4a, 0x1f, 0xd0, 0xfd, 0xe4, 
    0x64, 0x23, 0x12, 0x92, 0x3d, 0xab, 0x0c, 0x75, 0x16, 0xdc, 0xaf, 0x26, 
    0x8f, 0x17, 0x5c, 0xfa, 0x1f, 0xdf, 0x65, 0xf4, 0x68, 0xe5, 0xd4, 0x1a, 
    0x2e, 0xc8, 0xe7, 0x36, 0xb5, 0xa2, 0xca, 0xba, 0x90, 0xf7, 0xa6, 0x4b, 
    0x1f, 0xc9, 0x4f, 0xc4, 0x8f, 0xc8, 0x77, 0x4a, 0x5f, 0x3e, 0xe5, 0x1d, 
    0x7f, 0x82, 0xc2, 0x94, 0xa7, 0xc7, 0xe5, 0x48, 0x19, 0xc9, 0x73, 0x85, 
    0xcb, 0xa9, 0xe5, 0x4a, 0x77, 0x6c, 0x95, 0xa9, 0x34, 0xec, 0x56, 0x1b, 
    0xdf, 0xca, 0x1f, 0xa9, 0x19, 0xc5, 0x85, 0x74, 0x3e, 0xda, 0xa6, 0xe5, 
    0x60, 0x02, 0xa0, 0x02, 0x70, 0x69, 0xca, 0x32, 0x8f, 0x74, 0x97, 0xe9, 
    0x1b, 0x45, 0x29, 0x94, 0x6b, 0xf4, 0xd1, 0xed, 0x9b, 0x6f, 0x42, 0x09, 
    0x1a, 0x2d, 0xa6, 0xda, 0x2b, 0x10, 0xca, 0x3c, 0x1a, 0x8d, 0x6f, 0xa8, 
    0x50, 0x83, 0x27, 0x85, 0x78, 0x57, 0x91, 0x81, 0x3b, 0xa7, 0xc2, 0xc6, 
    0x50, 0x30, 0x4f, 0x9f, 0x19, 0xc0, 0xa5, 0xcd, 0xbc, 0x13, 0x03, 0x50, 
    0x6d, 0xc9, 0x2b, 0xb3, 0x77, 0x2a, 0x52, 0x0e, 0xd9, 0x94, 0xe5, 0x14, 
    0x1a, 0xd0, 0x03, 0xe3, 0xfb, 0xbb, 0xc7, 0x52, 0x39, 0x79, 0xfa, 0xb1, 
    0x21, 0xf5, 0xfa, 0xb6, 0x93, 0x5a, 0x36, 0x56, 0x18, 0xa1, 0x1d, 0xe4, 
    0x97, 0xd1, 0xb1, 0x8a, 0x54, 0x4a, 0xe6, 0xe9, 0x2e, 0x90, 0x2a, 0x88, 
    0x98, 0xac, 0xc6, 0xfe, 0x1d, 0x55, 0x73, 0xd4, 0xc6, 0xbe, 0x12, 0x7a, 
    0x7b, 0xfd, 0x95, 0xad, 0x3b, 0xd9, 0xdb, 0xf2, 0x9e, 0x95, 0xa9, 0x6c, 
    0x1d, 0x55, 0x27, 0xd4, 0xd5, 0x8f, 0x01, 0x52, 0x62, 0x54, 0xa4, 0x29, 
    0x2e, 0xf6, 0x57, 0x82, 0x97, 0x4f, 0x33, 0xef, 0x1d, 0x4d, 0x5f, 0xc4, 
    0xab, 0xa4, 0x0b, 0x02, 0x55, 0xd6, 0xbc, 0x81, 0x2b, 0xcb, 0x42, 0x14, 
    0x97, 0x57, 0xae, 0xf9, 0x1a, 0x7e, 0x45, 0xb4, 0xb8, 0xef, 0x2c, 0x32, 
    0x43, 0x9b, 0x05, 0x35, 0x2b, 0xad, 0x67, 0x1d, 0x65, 0xf2, 0x16, 0xd9, 
    0x18, 0xb0, 0x35, 0xb4, 0x98, 0x86, 0xa3, 0x9a, 0x4d, 0x0f, 0x5d, 0xa5, 
    0x21, 0x2d, 0x77, 0x36, 0x91, 0x77, 0xe3, 0xa6, 0x1c, 0x99, 0x5d, 0xeb, 
    0x24, 0x30, 0xc3, 0x75, 0x92, 0x5c, 0xe3, 0xf1, 0x11, 0xcb, 0x41, 0x68, 
    0x8c, 0x9a, 0x14, 0x97, 0x19, 0x9b, 0x65, 0xf4, 0x76, 0xab, 0x1b, 0x8a, 
    0x35, 0xde, 0xd9, 0xd9, 0x1a, 0xfa, 0x0a, 0x9e, 0x8f, 0x54, 0xf2, 0x4f, 
    0x1b, 0xdc, 0xb5, 0xf4, 0x6a, 0xe0, 0xa9, 0xc2, 0x3d, 0xd1, 0x1e, 0x35, 
    0x25, 0x70, 0xdb, 0xbc, 0x91, 0x9e, 0x55, 0x79, 0x1e, 0xd8, 0x29, 0xf4, 
    0x9b, 0xc2, 0xc2, 0xda, 0x43, 0x17, 0x20, 0xfd, 0x29, 0xfe, 0x6c, 0xa6, 
    0x95, 0xcf, 0xc9, 0xb8, 0x19, 0xf8, 0x43, 0xcf, 0x09, 0x82, 0x4f, 0x38, 
    0xb1, 0x63, 0x60, 0x2d, 0x37, 0xb1, 0x71, 0x8f, 0x18, 0x40, 0x2d, 0x0f, 
    0x1f, 0xe2, 0x51, 0x74, 0x8e, 0xa9, 0x08, 0x0c, 0x28, 0xfe, 0x87, 0xaf, 
    0xbc, 0x71, 0x56, 0x30, 0xb3, 0xd5, 0x5d, 0x58, 0x18, 0x7f, 0xf1, 0x50, 
    0x38, 0xdb, 0xf2, 0x7a, 0x43, 0xec, 0x79, 0x1d, 0x25, 0xe5, 0x1d, 0x4f, 
    0xb5, 0xf7, 0xe0, 0xa7, 0x1f, 0x75, 0x5f, 0x4b, 0xf1, 0x90, 0xb7, 0x2c, 
    0x91, 0xf3, 0x99, 0x00, 0x92, 0xf2, 0xe6, 0x50, 0xd6, 0x70, 0x20, 0x86, 
    0x1f, 0x6d, 0x21, 0x72, 0x66, 0x0b, 0x26, 0x1f, 0x11, 0xdc, 0x7f, 0xab, 
    0x87, 0x28, 0x42, 0x15, 0x4f, 0x95, 0x28, 0x0b, 0x1a, 0x40, 0x81, 0xed, 
    0x70, 0x2f, 0x18, 0x54, 0xe8, 0x14, 0x85, 0x7e, 0x6c, 0x28, 0x4a, 0xc8, 
    0xdb, 0x15, 0x25, 0x0a, 0x1a, 0x57, 0x81, 0xe4, 0x53, 0xff, 0xe0, 0x02, 
    0x32, 0xc0, 0xbd, 0x2d, 0x13, 0xaf, 0xc8, 0x96, 0x8f, 0xa6, 0x80, 0x00, 
    0x1a, 0x40, 0xc1, 0xdb, 0x33, 0xc6, 0xeb, 0x52, 0x47, 0xd0, 0x85, 0x61, 
    0x1d, 0xa9, 0x0a, 0x9e, 0x8f, 0x55, 0x99, 0x4a, 0x1a, 0x40, 0xdf, 0xd4, 
    0x76, 0x21, 0x43, 0x5a, 0x3a, 0xf2, 0xe7, 0x79, 0x1b, 0xfa, 0x39, 0xce, 
    0x8f, 0x1c, 0xad, 0x6b, 0x1a, 0x51, 0x9d, 0xd4, 0x68, 0xad, 0xfb, 0xca, 
    0x36, 0x7a, 0x02, 0x2d, 0x15, 0x6b, 0x96, 0xe6, 0x87, 0x7d, 0x5d, 0x11, 
    0x1d, 0xea, 0xa5, 0xd4, 0x07, 0xe2, 0x7a, 0x32, 0x1d, 0x7a, 0x08, 0x51, 
    0x0e, 0xbd, 0x54, 0xd4, 0x85, 0x5d, 0xed, 0x4b, 0x1b, 0xc1, 0x41, 0xe4, 
    0x2a, 0xe1, 0x8a, 0x82, 0x1a, 0xda, 0xcd, 0xf1, 0x0a, 0xfe, 0xc5, 0x3e, 
    0x84, 0x9b, 0x28, 0xc6, 0x1b, 0xc1, 0x1d, 0xf4, 0x2c, 0xf5, 0x45, 0xda, 
    0x03, 0xf0, 0x60, 0xa9, 0x0d, 0xf6, 0x63, 0xdc, 0x86, 0xb7, 0x6e, 0x54, 
    0x19, 0x4d, 0x41, 0xf4, 0x38, 0xd5, 0xfb, 0xb2, 0x1d, 0x7b, 0x3a, 0xc1, 
    0x0b, 0xf2, 0x2b, 0x0e, 0x0c, 0x23, 0xd5, 0x07, 0x1b, 0x51, 0x07, 0x95, 
    0x9e, 0x9c, 0x00, 0x79, 0xe2, 0xd0, 0x70, 0x00, 0x88, 0x20, 0x3a, 0xc0, 
    0xae, 0x61, 0x0a, 0x38, 0x1b, 0xd4, 0x25, 0x42, 0x89, 0xd8, 0x4b, 0x01, 
    0xd7, 0xc0, 0x57, 0x9d, 0x41, 0x67, 0x7f, 0x32, 0x87, 0x75, 0x61, 0x48, 
    0x18, 0x85, 0x77, 0x90, 0x10, 0x3f, 0x3c, 0x41, 0xfb, 0x7e, 0xa6, 0x2d, 
    0xe6, 0xce, 0x67, 0x3a, 0x92, 0x99, 0x42, 0x5a, 0x1f, 0x1a, 0x5b, 0xc3, 
    0xb0, 0x20, 0xc5, 0x45, 0x52, 0x55, 0xe8, 0x1b, 0xb4, 0xa6, 0xda, 0x85, 
    0xea, 0xbe, 0x72, 0xd2, 0x1f, 0x19, 0xc3, 0xd4, 0x37, 0xc8, 0xcb, 0x67, 
    0x1b, 0x7b, 0x46, 0x97, 0x46, 0x69, 0xbe, 0xee, 0x7c, 0x3a, 0xf4, 0x1b, 
    0x1f, 0x0b, 0xeb, 0xec, 0x19, 0x85, 0x72, 0x1e, 0x55, 0xe7, 0xd6, 0x87, 
    0x1d, 0x31, 0x94, 0x8d, 0x8d, 0xf0, 0x3f, 0x61, 0x1c, 0x83, 0x33, 0xed, 
    0x78, 0xd8, 0x2f, 0x85, 0xe0, 0xe7, 0x27, 0x78, 0xb3, 0x2b, 0xaa, 0xa6, 
    0x64, 0x90, 0x22, 0x1f, 0x1b, 0xdb, 0xdd, 0xf2, 0xb7, 0x94, 0x32, 0x41, 
    0xa4, 0x7c, 0xf0, 0x2c, 0xcd, 0xab, 0x03, 0x56, 0x67, 0x7b, 0xe7, 0xbc, 
    0x1b, 0xc7, 0xa5, 0xf3, 0x55, 0xce, 0xed, 0x31, 0xb4, 0x78, 0xf6, 0xd8, 
    0xdd, 0xfe, 0xe0, 0x26, 0x12, 0x74, 0xbf, 0x31, 0x1a, 0xd8, 0x2f, 0xf3, 
    0x97, 0xd9, 0x5f, 0x19, 0xc7, 0xe6, 0xa4, 0x7c, 0xec, 0xf6, 0xbc, 0x54, 
    0x75, 0x7c, 0x21, 0x4e, 0x1a, 0xd0, 0x67, 0xf3, 0x87, 0xb3, 0x74, 0xa9, 
    0xf4, 0x7b, 0x15, 0x27, 0xd1, 0xb2, 0x29, 0x0e, 0x7b, 0xb9, 0x28, 0x59, 
    0x1a, 0xcd, 0xc7, 0xf4, 0x07, 0xf3, 0x2a, 0x32, 0x0a, 0xd7, 0x0c, 0x8d, 
    0x02, 0xb0, 0x59, 0xc0, 0x83, 0xb5, 0xf7, 0x9e, 0x1a, 0xd0, 0x67, 0xf4, 
    0x3b, 0x86, 0xb5, 0xe2, 0x24, 0x4f, 0x35, 0x8d, 0x12, 0xc5, 0xbc, 0x6a, 
    0x8d, 0x1e, 0x7b, 0x42, 0x1f, 0x87, 0x25, 0xf4, 0x6a, 0x9d, 0x5a, 0xaa, 
    0x3d, 0x64, 0xf8, 0x7d, 0x25, 0xe6, 0x87, 0x8a, 0x99, 0x95, 0x71, 0x47, 
    0x1a, 0xc8, 0x67, 0xf4, 0xcb, 0xfd, 0x12, 0x7a, 0x6b, 0xe4, 0x5a, 0xbd, 
    0x3d, 0xff, 0x91, 0xf0, 0xa4, 0xf4, 0x32, 0xec, 0x1a, 0xc8, 0x25, 0xf5, 
    0x27, 0x5d, 0x26, 0xfa, 0xa7, 0x80, 0x65, 0x61, 0x51, 0x8d, 0xc8, 0x12, 
    0xa8, 0xb1, 0x8b, 0x06, 0x19, 0xc7, 0xd7, 0xd5, 0xa8, 0xb2, 0xcc, 0x3e, 
    0xcf, 0x10, 0x70, 0x27, 0x8f, 0x9d, 0xfe, 0x4e, 0xcb, 0xc4, 0xad, 0x42, 
    0x1b, 0x84, 0x51, 0xa0, 0x4e, 0xa1, 0x74, 0x3c, 0xc4, 0xc1, 0x6a, 0xaa, 
    0xda, 0x76, 0xe7, 0x91, 0xdc, 0xd7, 0xc1, 0xef, 0x1b, 0x80, 0x1d, 0xc4, 
    0xef, 0x65, 0x6d, 0xd4, 0xbe, 0x56, 0x70, 0x00, 0x03, 0xa0, 0x82, 0x0c, 
    0xbf, 0xd4, 0xee, 0x18, 0x19, 0xd9, 0xb1, 0xc7, 0xa9, 0xc3, 0x85, 0x51, 
    0x66, 0xd6, 0xa6, 0xe2, 0x83, 0xea, 0x57, 0x94, 0x0b, 0x65, 0x0a, 0xaf, 
    0x19, 0xc9, 0x43, 0xb4, 0x97, 0x36, 0x88, 0x13, 0xd5, 0x93, 0x91, 0xb0, 
    0xbc, 0xc3, 0xc6, 0x4e, 0x6b, 0x24, 0x09, 0xce, 0x19, 0xc3, 0x67, 0x42, 
    0x4e, 0xec, 0x08, 0x47, 0xef, 0x6c, 0x63, 0x7c, 0x02, 0x2a, 0x4a, 0x52, 
    0xda, 0x7a, 0x6b, 0xcb, 0x1d, 0xc0, 0x45, 0x5f, 0xf1, 0x7c, 0x5f, 0x7b, 
    0x6e, 0x41, 0x50, 0x2d, 0x14, 0x20, 0x01, 0xd6, 0x2f, 0x7a, 0x02, 0x36, 
    0x1b, 0xc5, 0x01, 0xc6, 0x3b, 0x67, 0x2e, 0xca, 0x19, 0x71, 0x1c, 0x33, 
    0x01, 0x21, 0x78, 0x15, 0xea, 0x40, 0x2b, 0x8a, 0x1b, 0xc0, 0x03, 0xa1, 
    0xdd, 0xae, 0x4d, 0xea, 0x5e, 0xe4, 0x1b, 0x09, 0xfb, 0xb3, 0xe3, 0x94, 
    0xa7, 0xbc, 0xec, 0x8d, 0x1b, 0xc0, 0x5f, 0xc6, 0x3f, 0x8f, 0x09, 0x16, 
    0x04, 0xea, 0x66, 0xe1, 0x67, 0x68, 0xb9, 0x0e, 0xb3, 0x1c, 0x32, 0xc5, 
    0x1b, 0xc3, 0xad, 0xd5, 0x94, 0x8a, 0x6a, 0xea, 0xbe, 0xd5, 0xf9, 0xcd, 
    0x5e, 0xe5, 0x8a, 0x30, 0xad, 0xd0, 0x2a, 0x8e, 0x1b, 0xc8, 0x1d, 0xd5, 
    0x75, 0xfa, 0xf9, 0xb2, 0xbd, 0x3a, 0x2d, 0xb1, 0x59, 0xbb, 0xef, 0x36, 
    0xb2, 0xf1, 0xac, 0xf3, 0x1f, 0x40, 0x47, 0xcd, 0x5e, 0x8b, 0x67, 0xa3, 
    0x1f, 0x58, 0x66, 0xfd, 0x61, 0xcf, 0x06, 0x8f, 0xa0, 0x00, 0xb9, 0x39, 
    0x1b, 0xd8, 0x25, 0x66, 0x2f, 0xa4, 0x55, 0x5a, 0xcf, 0x76, 0xb8, 0x2d, 
    0x8b, 0xcb, 0xb2, 0x31, 0xa3, 0xc5, 0x00, 0xa8, 0x18, 0x85, 0x35, 0x93, 
    0xdc, 0x7d, 0x4e, 0xcf, 0xce, 0x18, 0xa1, 0x21, 0x6c, 0x3f, 0xb9, 0x56, 
    0xb9, 0x7d, 0x36, 0xec, 0x1f, 0x00, 0x43, 0xc2, 0xf2, 0xe5, 0x59, 0x85, 
    0x08, 0x3f, 0xb2, 0x04, 0xac, 0x28, 0xad, 0x1b, 0x42, 0xd9, 0x08, 0xb2, 
    0x1f, 0x05, 0x7b, 0xdc, 0xb6, 0xff, 0x4f, 0x09, 0x66, 0x61, 0x41, 0x36, 
    0xae, 0xa2, 0x93, 0x27, 0xfd, 0xb9, 0x95, 0xb4, 0x1c, 0x83, 0x25, 0xe1, 
    0xf0, 0xc1, 0x20, 0x58, 0x0d, 0xd7, 0xf1, 0x1c, 0x73, 0xa0, 0xc7, 0x94, 
    0xd2, 0x76, 0x8b, 0x0a, 0x1c, 0x81, 0x2d, 0xf3, 0xca, 0x90, 0xf4, 0x31, 
    0x96, 0xfa, 0xcc, 0x81, 0x29, 0x22, 0x62, 0x10, 0x98, 0x95, 0x75, 0x87, 
    0x1c, 0x94, 0x3d, 0xfb, 0xd6, 0xc3, 0x17, 0x22, 0x8a, 0x89, 0xdf, 0x79, 
    0x52, 0x2a, 0x0e, 0x90, 0xab, 0xc7, 0xa2, 0xe3, 0x1f, 0x40, 0x47, 0xa4, 
    0xfe, 0xb1, 0x18, 0xf2, 0x92, 0x78, 0xa4, 0x22, 0x87, 0x28, 0xf8, 0xbe, 
    0xee, 0x2d, 0x2d, 0x19, 0x1f, 0x54, 0x7d, 0x76, 0x73, 0xfd, 0x1e, 0xe3, 
    0x45, 0x7d, 0x8b, 0x2d, 0x39, 0xea, 0x5b, 0xd6, 0x0b, 0xa3, 0xc2, 0xb5, 
    0x1f, 0x54, 0x7d, 0x5e, 0x0f, 0xab, 0x49, 0x9a, 0x53, 0xe3, 0x22, 0x65, 
    0x33, 0xf1, 0xbe, 0xc8, 0x17, 0xcb, 0xa7, 0xff, 0x1f, 0x54, 0x3d, 0x31, 
    0x69, 0x9b, 0xa8, 0x55, 0xae, 0x6c, 0xf5, 0xa9, 0x4b, 0x2a, 0xef, 0x92, 
    0x9c, 0xf0, 0xe1, 0x2a, 0x19, 0xc2, 0x37, 0x30, 0xe6, 0xb7, 0xfa, 0x9d, 
    0xbf, 0x7f, 0x80, 0x2d, 0x5f, 0xba, 0xae, 0x4f, 0xc6, 0x38, 0x9d, 0x4b, 
    0x19, 0xd8, 0x47, 0x24, 0xfb, 0x81, 0x79, 0xf1, 0x1d, 0xd7, 0x05, 0x80, 
    0x7b, 0x22, 0x4e, 0x4e, 0x9a, 0x75, 0x42, 0xac, 0x1c, 0x34, 0x39, 0x7a, 
    0xcf, 0x3f, 0x2e, 0x35, 0x63, 0xae, 0x29, 0x18, 0x28, 0x53, 0x76, 0x71, 
    0xff, 0x94, 0xa9, 0x4a, 0x18, 0xbd, 0x15, 0xdf, 0xdc, 0x29, 0x4a, 0x55, 
    0x5e, 0x54, 0xa5, 0x2c, 0xbf, 0x27, 0x3e, 0xa4, 0x09, 0x94, 0x7e, 0xe0, 
    0x1b, 0xa9, 0x71, 0xf4, 0xdd, 0x89, 0xcd, 0x7a, 0x82, 0x4b, 0x96, 0xad, 
    0x45, 0x24, 0xcc, 0xee, 0xa7, 0xf3, 0xbf, 0x60, 0x18, 0xb2, 0x83, 0xf5, 
    0x06, 0xef, 0x31, 0x82, 0x95, 0x95, 0xee, 0xe1, 0x4d, 0x2e, 0x79, 0xba, 
    0xa6, 0xe3, 0x60, 0xcf, 0x18, 0xbd, 0xdb, 0xf5, 0x3a, 0x1b, 0xc9, 0xc2, 
    0x9b, 0xd0, 0xa7, 0x51, 0x52, 0x2a, 0xab, 0x5c, 0xa5, 0x57, 0x0e, 0x25, 
    0x18, 0xbd, 0x97, 0xf5, 0x26, 0x71, 0xca, 0x0a, 0x9d, 0x9f, 0xc1, 0xe5, 
    0x56, 0xb9, 0x76, 0x68, 0xab, 0x1a, 0x8d, 0x9d, 0x18, 0xa7, 0x9b, 0xe5, 
    0x5a, 0xaf, 0x21, 0x7a, 0xa3, 0x63, 0x02, 0xf5, 0x51, 0x91, 0x93, 0x7e, 
    0xaa, 0xf6, 0xf2, 0x3e, 0x1c, 0x3f, 0x51, 0xe5, 0x5a, 0x96, 0x34, 0xd2, 
    0xbf, 0xc7, 0xa6, 0xb5, 0x59, 0xd5, 0x24, 0x6e, 0xb0, 0xfd, 0xc4, 0x6e, 
    0x18, 0xa1, 0x75, 0xd5, 0x9a, 0xbb, 0x8b, 0xaa, 0xc7, 0x8f, 0x24, 0xc5, 
    0x6a, 0x3e, 0x8e, 0x4a, 0xb3, 0x59, 0x30, 0x93, 0x19, 0xf0, 0x45, 0xcd, 
    0x27, 0xc1, 0x46, 0x52, 0xc3, 0x5a, 0xd7, 0x95, 0x69, 0xea, 0xef, 0xbb, 
    0xac, 0xf2, 0x60, 0x46, 0x1d, 0xf9, 0xa7, 0xc5, 0xd5, 0x90, 0xaa, 0x56, 
    0xd3, 0xed, 0x66, 0xe1, 0x6e, 0xf1, 0x1a, 0xb8, 0xbc, 0xf6, 0xfb, 0xb1, 
    0x1d, 0xfb, 0xc1, 0xb5, 0xe7, 0x90, 0x93, 0x32, 0xff, 0x57, 0x76, 0x9d, 
    0x85, 0xf5, 0x6a, 0xd6, 0x85, 0x32, 0x17, 0x15, 0x1a, 0xd8, 0x03, 0xd6, 
    0x12, 0xc9, 0xed, 0xf7, 0x2d, 0xed, 0xdc, 0x9d, 0x2b, 0x68, 0x78, 0x8c, 
    0xc7, 0x7b, 0x50, 0x80, 0x1b, 0xcc, 0xb5, 0xee, 0x77, 0x78, 0x8b, 0x1b, 
    0x2a, 0x79, 0xda, 0xbb, 0xa9, 0xc0, 0xb7, 0xbc, 0xc9, 0x5d, 0xb5, 0x4a, 
    0x1b, 0xcf, 0xb1, 0xe6, 0x56, 0xa9, 0x53, 0x83, 0x34, 0x55, 0x26, 0x21, 
    0x9f, 0xc8, 0xda, 0x23, 0xdb, 0xaa, 0x99, 0x14, 0x1b, 0xc0, 0x25, 0xd6, 
    0x6c, 0xce, 0x96, 0x53, 0x39, 0xd0, 0x08, 0x3f, 0xbc, 0xe8, 0x62, 0x8e, 
    0xf3, 0xc7, 0x1d, 0x51, 0x19, 0x94, 0x37, 0xa7, 0x3e, 0x9b, 0x2f, 0xa3, 
    0x2d, 0xf6, 0xb8, 0xb1, 0xb1, 0x2b, 0x80, 0x41, 0xe6, 0x14, 0xa5, 0xb6, 
    0x1b, 0xdf, 0xeb, 0xc6, 0xbb, 0xa1, 0x57, 0xc3, 0x5c, 0x6b, 0x9d, 0x79, 
    0x9e, 0xf5, 0x84, 0x4e, 0xc8, 0xf8, 0x9f, 0x6b, 0x1b, 0xdf, 0xe3, 0xd6, 
    0x64, 0xdc, 0x4b, 0x5b, 0x1e, 0xb4, 0xac, 0x4f, 0xa9, 0x2a, 0x8a, 0x14, 
    0xc5, 0x59, 0xf6, 0x04, 0x1b, 0xdf, 0xeb, 0xe6, 0x26, 0x66, 0x3c, 0x3b, 
    0x1e, 0xf5, 0xd1, 0xc9, 0x8d, 0xf1, 0x70, 0xa6, 0xc4, 0xfe, 0x63, 0x34, 
    0x1b, 0xdf, 0xf7, 0xde, 0xc7, 0xd8, 0xd1, 0xc0, 0x05, 0x7a, 0xbe, 0x41, 
    0xa1, 0x40, 0x7a, 0x21, 0xe1, 0xd2, 0x31, 0xab, 0x1f, 0x91, 0xd5, 0xc7, 
    0x17, 0xe3, 0x9a, 0x63, 0x8d, 0xe1, 0x5d, 0x3b, 0xa3, 0x84, 0x96, 0x11, 
    0x9d, 0xb2, 0xf7, 0x31, 0x1a, 0x15, 0x0f, 0xdb, 0xf1, 0xae, 0xbe, 0x96, 
    0x46, 0xc2, 0x00, 0x62, 0x3b, 0x6b, 0xe4, 0xbd, 0x28, 0x93, 0x2c, 0xe8, 
    0x1f, 0x91, 0xf5, 0xc5, 0x0f, 0xe9, 0x6c, 0xf3, 0x56, 0x76, 0xd2, 0xbd, 
    0xc2, 0x64, 0x1b, 0x7b, 0x87, 0x9f, 0xe3, 0x04, 0x1b, 0xcd, 0x1b, 0xcc, 
    0xfc, 0xaa, 0xd7, 0x03, 0x73, 0xd7, 0x3c, 0x3d, 0x86, 0x20, 0xd0, 0x06, 
    0xf9, 0xe7, 0xa9, 0x73, 0x18, 0xaf, 0xb5, 0xc5, 0xf7, 0x54, 0x89, 0x7a, 
    0xf3, 0xf2, 0x23, 0x9f, 0xc6, 0x72, 0x16, 0x44, 0xb6, 0xb0, 0x1f, 0x8b, 
    0x1a, 0xd8, 0x7d, 0xe5, 0xb8, 0x47, 0x6f, 0x8a, 0xd2, 0x4d, 0x96, 0xe1, 
    0x76, 0x31, 0x19, 0x5b, 0x7f, 0x9f, 0x41, 0xcb, 0x1a, 0x51, 0xa1, 0xc3, 
    0x86, 0x83, 0x85, 0x3a, 0xf2, 0xd5, 0x65, 0x93, 0x71, 0x69, 0x0c, 0x55, 
    0x05, 0x34, 0x27, 0x49, 0x1d, 0x54, 0x13, 0xb6, 0x4f, 0xa1, 0x4f, 0x9b, 
    0x23, 0xe3, 0x8e, 0x85, 0x91, 0xf6, 0x99, 0x1b, 0xd2, 0xe6, 0xb7, 0x45, 
    0x1a, 0x7a, 0x35, 0xb6, 0x68, 0xbe, 0x6f, 0x83, 0x17, 0xae, 0x03, 0x65, 
    0x9b, 0xb5, 0xeb, 0xf2, 0xd1, 0x05, 0x66, 0xcb, 0x1f, 0x79, 0x03, 0x86, 
    0x93, 0xcb, 0x0b, 0xbb, 0x69, 0xf2, 0x94, 0x88, 0x88, 0x29, 0x5f, 0x46, 
    0xd7, 0xc4, 0xac, 0x99, 0x1f, 0x71, 0x7d, 0x46, 0xde, 0xd4, 0xeb, 0x00, 
    0x02, 0xd7, 0x74, 0xe7, 0x67, 0xa9, 0x29, 0x95, 0x8d, 0xbb, 0x67, 0x68, 
    0x19, 0xc3, 0x07, 0x36, 0xbe, 0xa3, 0x0f, 0xeb, 0x1f, 0x5f, 0x37, 0x3c, 
    0x6c, 0xbc, 0xb9, 0x24, 0x37, 0x7f, 0xa1, 0xd0, 0x19, 0xd4, 0x07, 0x30, 
    0x47, 0xdd, 0xd3, 0x0c, 0x57, 0xf5, 0x8c, 0xe1, 0x9b, 0xb7, 0x99, 0x6a, 
    0xa4, 0xbc, 0x66, 0x09, 0x1a, 0x60, 0x31, 0x9f, 0xdb, 0x0d, 0xce, 0x74, 
    0x99, 0x06, 0xe2, 0x77, 0xcc, 0x0a, 0x56, 0x15, 0x96, 0x55, 0x28, 0xe0, 
    0x19, 0x80, 0x03, 0xe5, 0x9c, 0x1c, 0x00, 0x00, 0xa3, 0xcf, 0x95, 0x4a, 
    0xae, 0xec, 0x7a, 0xac, 0x8f, 0x31, 0x4a, 0x6a, 0x1f, 0x5f, 0xa1, 0xd6, 
    0x1b, 0xa9, 0x53, 0x83, 0x09, 0x54, 0x1a, 0xb5, 0x93, 0xea, 0xc1, 0x2c, 
    0xca, 0xf9, 0x54, 0xc7, 0x1b, 0xc0, 0x15, 0xb5, 0xef, 0xcb, 0x0a, 0xbf, 
    0x9f, 0x50, 0xa4, 0x54, 0xb9, 0x56, 0x42, 0x90, 0xf3, 0xcb, 0x07, 0xf2, 
    0x1f, 0x68, 0x25, 0x57, 0x80, 0xa9, 0x0a, 0x57, 0xa1, 0x51, 0xd6, 0xd0, 
    0xaa, 0x36, 0x3e, 0x0e, 0x3f, 0x72, 0x57, 0x19, 0x19, 0xd8, 0x67, 0x40, 
    0xe7, 0xa7, 0xa9, 0x10, 0xb3, 0xd3, 0xc5, 0xa4, 0x0b, 0xe5, 0x5a, 0xb2, 
    0x62, 0x73, 0x38, 0x35, 0x19, 0xc9, 0x87, 0x32, 0x6b, 0xde, 0xfe, 0x7d, 
    0x38, 0xf3, 0xbf, 0x2a, 0xe7, 0x28, 0x54, 0xbe, 0xbc, 0xb4, 0x15, 0x51, 
    0x1b, 0x99, 0xdd, 0x6c, 0x1c, 0x7d, 0x59, 0xce, 0xee, 0x32, 0x7c, 0xe5, 
    0x84, 0x2b, 0x00, 0x05, 0x12, 0x14, 0xa6, 0xa0, 0x1a, 0xd8, 0x25, 0xb2, 
    0xb0, 0xcf, 0x32, 0x8d, 0xd2, 0x2d, 0xbe, 0xc8, 0x80, 0x8a, 0x52, 0x80, 
    0xce, 0x15, 0xa9, 0x08, 0x1f, 0xc4, 0xdd, 0xf3, 0xf0, 0xa9, 0x32, 0x83, 
    0x03, 0x9e, 0xab, 0x31, 0x81, 0xea, 0x5f, 0x97, 0xcf, 0x76, 0xcc, 0x88, 
    0x1a, 0xc3, 0x97, 0xf6, 0x38, 0xf8, 0x5e, 0x93, 0x3f, 0xeb, 0xe5, 0xd9, 
    0xa3, 0xe8, 0x6a, 0x94, 0xd1, 0x92, 0xa6, 0xdf, 0x1e, 0xd3, 0x55, 0xf6, 
    0x9b, 0xa1, 0x28, 0x1c, 0x2f, 0xd4, 0xac, 0x25, 0xae, 0xa8, 0x78, 0x8a, 
    0xd5, 0x7a, 0xb3, 0x85, 0x1a, 0xc3, 0xab, 0xf6, 0xb7, 0xad, 0x6b, 0xcb, 
    0x43, 0x74, 0xe6, 0x05, 0xb1, 0x2f, 0x11, 0x16, 0xdd, 0x6f, 0x2c, 0xfb, 
    0x1b, 0xdc, 0xa9, 0xf6, 0xff, 0xf1, 0xb9, 0xbb, 0x75, 0x57, 0x9a, 0x59, 
    0xbe, 0xeb, 0x5d, 0x58, 0xe7, 0xf7, 0x88, 0x37, 0x19, 0x9c, 0xc5, 0xf8, 
    0xee, 0xbf, 0xde, 0x45, 0xae, 0xfa, 0x08, 0xa5, 0xea, 0xe5, 0x24, 0xac, 
    0xf0, 0xf8, 0x6b, 0x44, 0x1b, 0xc3, 0x85, 0xe2, 0x22, 0xdd, 0x28, 0x43, 
    0xc5, 0xd2, 0x27, 0xa8, 0x11, 0xff, 0x42, 0x14, 0xfa, 0xf7, 0x36, 0xb5, 
    0x1b, 0xd8, 0x33, 0xb7, 0xb3, 0xe6, 0xeb, 0x54, 0x27, 0xd7, 0x54, 0x2e, 
    0x07, 0x80, 0x2a, 0xac, 0xf8, 0xbc, 0xed, 0x7e, 0x1b, 0xc3, 0x53, 0xb7, 
    0xe5, 0x15, 0x51, 0xfc, 0x23, 0xe1, 0x05, 0x7a, 0x2b, 0xc8, 0x52, 0x31, 
    0x11, 0x77, 0x5f, 0x0b, 0x1b, 0xdd, 0x01, 0xbf, 0xf1, 0x69, 0x2b, 0x84, 
    0x6c, 0x00, 0x00, 0x00, 0xa7, 0x97, 0xce, 0x71, 0x20, 0x91, 0x3d, 0x45, 
    0x18, 0x81, 0x39, 0xca, 0x69, 0x1b, 0x27, 0x5c, 0x88, 0x3e, 0x8f, 0xca, 
    0x45, 0x3a, 0x6a, 0x57, 0x2b, 0x65, 0x11, 0x08, 0x18, 0x81, 0x5b, 0xe9, 
    0x2c, 0xa9, 0x8c, 0x54, 0x53, 0x90, 0x85, 0x26, 0x62, 0xaf, 0x77, 0xa1, 
    0x2b, 0xa4, 0xfd, 0x4d, 0x1f, 0x41, 0x73, 0xb9, 0xa7, 0xa2, 0x47, 0xc3, 
    0x87, 0xff, 0xd3, 0xae, 0x47, 0x87, 0x42, 0x03, 0x24, 0xb0, 0xcf, 0xc7, 
    0x1f, 0x41, 0x2b, 0xb8, 0xcf, 0x3f, 0x02, 0x74, 0x75, 0xf7, 0x93, 0xe4, 
    0x94, 0xe4, 0xaf, 0x9f, 0x21, 0x95, 0x80, 0xe7, 0x1b, 0xce, 0x1b, 0xa8, 
    0xe5, 0xb5, 0x6b, 0x9c, 0x92, 0x92, 0xf6, 0x82, 0x4d, 0xed, 0xc8, 0xb7, 
    0x2d, 0xfc, 0x1b, 0xee, 0x19, 0xd9, 0xa7, 0x40, 0x0b, 0x82, 0xa9, 0xf9, 
    0xbf, 0x50, 0xaf, 0xfd, 0xdf, 0x98, 0x43, 0x51, 0x88, 0x03, 0xa9, 0x29, 
    0x19, 0xd8, 0x67, 0x29, 0x31, 0xc9, 0x9f, 0xb1, 0xad, 0xd3, 0xb3, 0x20, 
    0x44, 0xb1, 0x64, 0xf8, 0x2d, 0x72, 0xa9, 0x77, 0x19, 0xd4, 0x7d, 0x26, 
    0x6e, 0xff, 0xc4, 0x54, 0x59, 0xe2, 0xdf, 0xb1, 0xbf, 0xb6, 0x9b, 0x19, 
    0x99, 0xdf, 0xc0, 0x6d, 0x19, 0xd4, 0x47, 0x1f, 0x8b, 0xe1, 0x78, 0x22, 
    0x9d, 0xe4, 0xfd, 0x3d, 0x91, 0xe3, 0x7e, 0xc2, 0x03, 0xdc, 0x05, 0x29, 
    0x19, 0xd4, 0x7d, 0x2e, 0x31, 0xad, 0x9f, 0xbf, 0xe8, 0x85, 0xa4, 0xdc, 
    0x44, 0xb4, 0xb5, 0x60, 0x53, 0x7b, 0x01, 0x18, 0x19, 0xc1, 0xe3, 0x2a, 
    0xb0, 0xea, 0xe9, 0x5a, 0x49, 0x6c, 0x95, 0xa8, 0x6f, 0xb6, 0xf3, 0xfc, 
    0x2c, 0xb0, 0x49, 0x28, 0x19, 0xd8, 0x67, 0x2c, 0xc9, 0xbc, 0xe6, 0x8a, 
    0xf8, 0x50, 0xac, 0x28, 0x64, 0xb9, 0x57, 0xee, 0x62, 0x74, 0xa2, 0x35, 
    0x19, 0xcd, 0x07, 0x20, 0xac, 0xc2, 0x72, 0x15, 0x67, 0x60, 0xb9, 0x80, 
    0xe3, 0xb9, 0x8e, 0x44, 0xee, 0x3e, 0x65, 0x8b, 0x19, 0xd8, 0x67, 0x2f, 
    0xcd, 0xe3, 0xef, 0xc9, 0xd9, 0x56, 0x0f, 0xd8, 0x5a, 0x27, 0xbb, 0xc0, 
    0x19, 0x13, 0x2f, 0xe6, 0x19, 0xd8, 0x67, 0x24, 0xe7, 0x8a, 0x49, 0xc9, 
    0xe5, 0x41, 0x6e, 0xd6, 0x01, 0xcc, 0x3b, 0x3f, 0x6a, 0x77, 0x5f, 0x97, 
    0x19, 0xd0, 0x67, 0x25, 0xde, 0xae, 0x19, 0x2e, 0xcf, 0x6f, 0x99, 0x3f, 
    0xdf, 0xab, 0xaf, 0xed, 0xd0, 0x1b, 0x81, 0xca, 0x19, 0xd8, 0x67, 0x1a, 
    0x9c, 0xd9, 0x41, 0x51, 0x38, 0x40, 0x80, 0x58, 0x19, 0xe4, 0x75, 0x1e, 
    0xd3, 0x50, 0x7f, 0xcc, 0x19, 0xd8, 0x73, 0x25, 0xb8, 0xd2, 0x13, 0x7f, 
    0xbe, 0xd0, 0xbc, 0x02, 0x55, 0x7f, 0x4c, 0xe2, 0x9d, 0xf6, 0xb8, 0x6c, 
    0x19, 0xd8, 0x6f, 0x2b, 0x6e, 0x8c, 0xea, 0x1a, 0xe7, 0x5f, 0x3e, 0x25, 
    0x41, 0x76, 0x1f, 0x41, 0xa0, 0x6c, 0xa5, 0x2b, 0x19, 0xdd, 0x47, 0x23, 
    0xf2, 0xe7, 0x0b, 0x53, 0x9f, 0xf2, 0xea, 0x91, 0x2a, 0xf6, 0x13, 0xda, 
    0x35, 0xcf, 0xd7, 0xf6, 0x19, 0xd9, 0xd7, 0x2f, 0xa5, 0x25, 0xe7, 0xc3, 
    0x7e, 0x73, 0x52, 0xbc, 0x06, 0xff, 0x1f, 0xf0, 0xd5, 0xd5, 0x60, 0xb3, 
    0x19, 0xc3, 0x07, 0x2f, 0x31, 0xa6, 0x95, 0x55, 0xdf, 0x7f, 0x6a, 0x99, 
    0x6f, 0xb3, 0xfc, 0x2b, 0x2f, 0x9b, 0x51, 0x80, 0x19, 0xd4, 0x57, 0x21, 
    0x50, 0xe4, 0x5f, 0xe8, 0x53, 0xe2, 0xf7, 0x70, 0x07, 0xb3, 0x7d, 0xb9, 
    0xb0, 0x17, 0x9d, 0x04, 0x19, 0xc3, 0x07, 0x2e, 0x51, 0xf2, 0x6f, 0xc6, 
    0xb6, 0xd3, 0xa6, 0x9f, 0x40, 0xf6, 0xb5, 0xc3, 0xf9, 0xb3, 0x1d, 0x47, 
    0x19, 0xd4, 0x67, 0x2f, 0x7c, 0xae, 0x70, 0x01, 0xed, 0xd0, 0xba, 0x29, 
    0x3d, 0x73, 0xb2, 0xd5, 0x4b, 0x53, 0x57, 0xc6, 0x19, 0xd4, 0x67, 0x2a, 
    0x5b, 0x93, 0x36, 0x1d, 0xdf, 0xce, 0xac, 0xd0, 0xf9, 0x65, 0x4b, 0x44, 
    0x45, 0xd1, 0x9d, 0x12, 0x19, 0xd8, 0x73, 0x29, 0x23, 0xd9, 0x2f, 0x5f, 
    0x3e, 0x54, 0xbc, 0x28, 0x54, 0xb8, 0x59, 0xfe, 0xa3, 0x7f, 0x57, 0xe1, 
    0x19, 0xd4, 0x07, 0x26, 0xa8, 0xee, 0xbf, 0x84, 0x22, 0xe1, 0x92, 0x73, 
    0xb4, 0xf6, 0x17, 0xc4, 0x97, 0xd4, 0xb9, 0x25, 0x19, 0xc1, 0xdd, 0x2c, 
    0x3e, 0xac, 0x94, 0x96, 0x32, 0xe5, 0x08, 0x3c, 0x8b, 0xf2, 0xa7, 0xf0, 
    0xf1, 0x54, 0xe5, 0x1f, 0x19, 0xc1, 0xe7, 0x12, 0x89, 0xa6, 0xa7, 0x47, 
    0xcd, 0xde, 0xc1, 0x2b, 0x4c, 0xa8, 0xf2, 0xaf, 0xe8, 0xf8, 0x41, 0x5b, 
    0x19, 0xd4, 0x67, 0x2d, 0x6d, 0xeb, 0x01, 0xaf, 0x06, 0xfe, 0x88, 0xa5, 
    0x13, 0xff, 0x6d, 0xee, 0xcd, 0xdd, 0x3d, 0x95, 0x19, 0xd4, 0x67, 0x21, 
    0x92, 0xc6, 0xcd, 0x7a, 0x8c, 0x7e, 0x23, 0x55, 0x43, 0xfa, 0x4a, 0x6a, 
    0x3b, 0xd2, 0xc4, 0x34, 0x19, 0xd4, 0x67, 0x24, 0x1e, 0xc2, 0x03, 0xf2, 
    0xac, 0xcb, 0xef, 0x32, 0xd7, 0x0a, 0x3a, 0xb0, 0x02, 0x56, 0xdc, 0xa2, 
    0x19, 0xc0, 0x07, 0x21, 0x72, 0xd7, 0xe9, 0xb8, 0x7c, 0x4d, 0x37, 0x9c, 
    0x69, 0xb8, 0x5e, 0xbd, 0x75, 0xfd, 0xab, 0x2d, 0x19, 0xd9, 0xe3, 0x27, 
    0xfe, 0xa2, 0x27, 0xe4, 0xe8, 0xe0, 0xc0, 0xd0, 0x19, 0x48, 0x42, 0xc1, 
    0xf1, 0x31, 0x0e, 0xcb, 0x19, 0xc0, 0x67, 0x21, 0x3b, 0xe6, 0xb5, 0x82, 
    0x09, 0x64, 0x04, 0x2c, 0x96, 0x2a, 0x59, 0x9d, 0x04, 0xfb, 0x27, 0x38, 
    0x19, 0xc1, 0xdd, 0x26, 0xd8, 0xbd, 0x93, 0x15, 0x33, 0xd3, 0xb4, 0x8c, 
    0x04, 0xeb, 0xbe, 0x11, 0x95, 0xbf, 0xd4, 0x47, 0x19, 0xdd, 0x27, 0x23, 
    0x0f, 0xbd, 0x4b, 0x50, 0xa2, 0xfe, 0x8b, 0xdd, 0x86, 0x78, 0x42, 0xd4, 
    0x3e, 0x7a, 0xe5, 0xd9, 0x19, 0xd0, 0x27, 0x10, 0x50, 0xad, 0x48, 0x51, 
    0x63, 0x7f, 0x5f, 0x28, 0x13, 0xb9, 0x86, 0xcc, 0x6f, 0xdf, 0xd5, 0xae, 
    0x19, 0xd9, 0x87, 0x16, 0x29, 0xec, 0x3f, 0x13, 0xe4, 0xe7, 0x1c, 0x65, 
    0xda, 0xe5, 0x6b, 0x37, 0x82, 0x13, 0x81, 0x4b, 0x19, 0xc0, 0x67, 0x22, 
    0x3b, 0xa7, 0x2b, 0x59, 0x8e, 0xd0, 0xbf, 0x79, 0x59, 0xb1, 0xcc, 0xd0, 
    0x51, 0xdf, 0x3c, 0x6d, 0x19, 0xc0, 0x5d, 0x2d, 0xa1, 0x8f, 0x2d, 0x45, 
    0x3e, 0x7c, 0xb5, 0x3f, 0x77, 0x2a, 0x42, 0x14, 0x80, 0xb5, 0x41, 0x31, 
    0x1b, 0x89, 0x79, 0x10, 0x33, 0xa2, 0x76, 0x59, 0x50, 0x30, 0x77, 0x39, 
    0x78, 0x0e, 0x73, 0x9d, 0x14, 0x07, 0x39, 0xce
};