#include "waves.h"

const uint16_t sine_10khz[SINE_10KHZ_NUM] = {
    2048, 2112, 2176, 2240, 2302, 2364, 2425, 2484, 2541, 2596, 
    2649, 2700, 2748, 2794, 2836, 2876, 2912, 2944, 2974, 2999, 
    3021, 3039, 3053, 3063, 3069, 3071, 3069, 3063, 3053, 3039, 
    3021, 2999, 2974, 2944, 2912, 2876, 2836, 2794, 2748, 2700, 
    2649, 2596, 2541, 2484, 2425, 2364, 2302, 2240, 2176, 2112, 
    2048, 1984, 1920, 1856, 1794, 1732, 1671, 1612, 1555, 1500, 
    1447, 1396, 1348, 1302, 1260, 1220, 1184, 1152, 1122, 1097, 
    1075, 1057, 1043, 1033, 1027, 1025, 1027, 1033, 1043, 1057, 
    1075, 1097, 1122, 1152, 1184, 1220, 1260, 1302, 1348, 1396, 
    1447, 1500, 1555, 1612, 1671, 1732, 1794, 1856, 1920, 1984, 
};

const uint16_t sine_1khz[SINE_1KHZ_NUM] = {
    2048, 2054, 2061, 2067, 2074, 2080, 2087, 2093, 2099, 2106, 
    2112, 2119, 2125, 2131, 2138, 2144, 2151, 2157, 2163, 2170, 
    2176, 2183, 2189, 2195, 2202, 2208, 2214, 2221, 2227, 2233, 
    2240, 2246, 2252, 2259, 2265, 2271, 2277, 2284, 2290, 2296, 
    2302, 2309, 2315, 2321, 2327, 2333, 2340, 2346, 2352, 2358, 
    2364, 2370, 2376, 2382, 2388, 2395, 2401, 2407, 2413, 2419, 
    2425, 2431, 2437, 2442, 2448, 2454, 2460, 2466, 2472, 2478, 
    2484, 2489, 2495, 2501, 2507, 2512, 2518, 2524, 2530, 2535, 
    2541, 2546, 2552, 2558, 2563, 2569, 2574, 2580, 2585, 2591, 
    2596, 2602, 2607, 2612, 2618, 2623, 2628, 2634, 2639, 2644, 
    2649, 2654, 2660, 2665, 2670, 2675, 2680, 2685, 2690, 2695, 
    2700, 2705, 2710, 2715, 2720, 2725, 2729, 2734, 2739, 2744, 
    2748, 2753, 2758, 2762, 2767, 2771, 2776, 2780, 2785, 2789, 
    2794, 2798, 2802, 2807, 2811, 2815, 2820, 2824, 2828, 2832, 
    2836, 2840, 2844, 2848, 2852, 2856, 2860, 2864, 2868, 2872, 
    2876, 2879, 2883, 2887, 2890, 2894, 2898, 2901, 2905, 2908, 
    2912, 2915, 2919, 2922, 2925, 2929, 2932, 2935, 2938, 2941, 
    2944, 2948, 2951, 2954, 2957, 2959, 2962, 2965, 2968, 2971, 
    2974, 2976, 2979, 2982, 2984, 2987, 2989, 2992, 2994, 2997, 
    2999, 3002, 3004, 3006, 3008, 3011, 3013, 3015, 3017, 3019, 
    3021, 3023, 3025, 3027, 3029, 3030, 3032, 3034, 3036, 3037, 
    3039, 3040, 3042, 3043, 3045, 3046, 3048, 3049, 3050, 3052, 
    3053, 3054, 3055, 3056, 3057, 3058, 3059, 3060, 3061, 3062, 
    3063, 3064, 3064, 3065, 3066, 3066, 3067, 3068, 3068, 3069, 
    3069, 3069, 3070, 3070, 3070, 3070, 3071, 3071, 3071, 3071, 
    3071, 3071, 3071, 3071, 3071, 3070, 3070, 3070, 3070, 3069, 
    3069, 3069, 3068, 3068, 3067, 3066, 3066, 3065, 3064, 3064, 
    3063, 3062, 3061, 3060, 3059, 3058, 3057, 3056, 3055, 3054, 
    3053, 3052, 3050, 3049, 3048, 3046, 3045, 3043, 3042, 3040, 
    3039, 3037, 3036, 3034, 3032, 3030, 3029, 3027, 3025, 3023, 
    3021, 3019, 3017, 3015, 3013, 3011, 3008, 3006, 3004, 3002, 
    2999, 2997, 2994, 2992, 2989, 2987, 2984, 2982, 2979, 2976, 
    2974, 2971, 2968, 2965, 2962, 2959, 2957, 2954, 2951, 2948, 
    2944, 2941, 2938, 2935, 2932, 2929, 2925, 2922, 2919, 2915, 
    2912, 2908, 2905, 2901, 2898, 2894, 2890, 2887, 2883, 2879, 
    2876, 2872, 2868, 2864, 2860, 2856, 2852, 2848, 2844, 2840, 
    2836, 2832, 2828, 2824, 2820, 2815, 2811, 2807, 2802, 2798, 
    2794, 2789, 2785, 2780, 2776, 2771, 2767, 2762, 2758, 2753, 
    2748, 2744, 2739, 2734, 2729, 2725, 2720, 2715, 2710, 2705, 
    2700, 2695, 2690, 2685, 2680, 2675, 2670, 2665, 2660, 2654, 
    2649, 2644, 2639, 2634, 2628, 2623, 2618, 2612, 2607, 2602, 
    2596, 2591, 2585, 2580, 2574, 2569, 2563, 2558, 2552, 2546, 
    2541, 2535, 2530, 2524, 2518, 2512, 2507, 2501, 2495, 2489, 
    2484, 2478, 2472, 2466, 2460, 2454, 2448, 2442, 2437, 2431, 
    2425, 2419, 2413, 2407, 2401, 2395, 2388, 2382, 2376, 2370, 
    2364, 2358, 2352, 2346, 2340, 2333, 2327, 2321, 2315, 2309, 
    2302, 2296, 2290, 2284, 2277, 2271, 2265, 2259, 2252, 2246, 
    2240, 2233, 2227, 2221, 2214, 2208, 2202, 2195, 2189, 2183, 
    2176, 2170, 2163, 2157, 2151, 2144, 2138, 2131, 2125, 2119, 
    2112, 2106, 2099, 2093, 2087, 2080, 2074, 2067, 2061, 2054, 
    2048, 2042, 2035, 2029, 2022, 2016, 2009, 2003, 1997, 1990, 
    1984, 1977, 1971, 1965, 1958, 1952, 1945, 1939, 1933, 1926, 
    1920, 1913, 1907, 1901, 1894, 1888, 1882, 1875, 1869, 1863, 
    1856, 1850, 1844, 1837, 1831, 1825, 1819, 1812, 1806, 1800, 
    1794, 1787, 1781, 1775, 1769, 1763, 1756, 1750, 1744, 1738, 
    1732, 1726, 1720, 1714, 1708, 1701, 1695, 1689, 1683, 1677, 
    1671, 1665, 1659, 1654, 1648, 1642, 1636, 1630, 1624, 1618, 
    1612, 1607, 1601, 1595, 1589, 1584, 1578, 1572, 1566, 1561, 
    1555, 1550, 1544, 1538, 1533, 1527, 1522, 1516, 1511, 1505, 
    1500, 1494, 1489, 1484, 1478, 1473, 1468, 1462, 1457, 1452, 
    1447, 1442, 1436, 1431, 1426, 1421, 1416, 1411, 1406, 1401, 
    1396, 1391, 1386, 1381, 1376, 1371, 1367, 1362, 1357, 1352, 
    1348, 1343, 1338, 1334, 1329, 1325, 1320, 1316, 1311, 1307, 
    1302, 1298, 1294, 1289, 1285, 1281, 1276, 1272, 1268, 1264, 
    1260, 1256, 1252, 1248, 1244, 1240, 1236, 1232, 1228, 1224, 
    1220, 1217, 1213, 1209, 1206, 1202, 1198, 1195, 1191, 1188, 
    1184, 1181, 1177, 1174, 1171, 1167, 1164, 1161, 1158, 1155, 
    1152, 1148, 1145, 1142, 1139, 1137, 1134, 1131, 1128, 1125, 
    1122, 1120, 1117, 1114, 1112, 1109, 1107, 1104, 1102, 1099, 
    1097, 1094, 1092, 1090, 1088, 1085, 1083, 1081, 1079, 1077, 
    1075, 1073, 1071, 1069, 1067, 1066, 1064, 1062, 1060, 1059, 
    1057, 1056, 1054, 1053, 1051, 1050, 1048, 1047, 1046, 1044, 
    1043, 1042, 1041, 1040, 1039, 1038, 1037, 1036, 1035, 1034, 
    1033, 1032, 1032, 1031, 1030, 1030, 1029, 1028, 1028, 1027, 
    1027, 1027, 1026, 1026, 1026, 1026, 1025, 1025, 1025, 1025, 
    1025, 1025, 1025, 1025, 1025, 1026, 1026, 1026, 1026, 1027, 
    1027, 1027, 1028, 1028, 1029, 1030, 1030, 1031, 1032, 1032, 
    1033, 1034, 1035, 1036, 1037, 1038, 1039, 1040, 1041, 1042, 
    1043, 1044, 1046, 1047, 1048, 1050, 1051, 1053, 1054, 1056, 
    1057, 1059, 1060, 1062, 1064, 1066, 1067, 1069, 1071, 1073, 
    1075, 1077, 1079, 1081, 1083, 1085, 1088, 1090, 1092, 1094, 
    1097, 1099, 1102, 1104, 1107, 1109, 1112, 1114, 1117, 1120, 
    1122, 1125, 1128, 1131, 1134, 1137, 1139, 1142, 1145, 1148, 
    1152, 1155, 1158, 1161, 1164, 1167, 1171, 1174, 1177, 1181, 
    1184, 1188, 1191, 1195, 1198, 1202, 1206, 1209, 1213, 1217, 
    1220, 1224, 1228, 1232, 1236, 1240, 1244, 1248, 1252, 1256, 
    1260, 1264, 1268, 1272, 1276, 1281, 1285, 1289, 1294, 1298, 
    1302, 1307, 1311, 1316, 1320, 1325, 1329, 1334, 1338, 1343, 
    1348, 1352, 1357, 1362, 1367, 1371, 1376, 1381, 1386, 1391, 
    1396, 1401, 1406, 1411, 1416, 1421, 1426, 1431, 1436, 1442, 
    1447, 1452, 1457, 1462, 1468, 1473, 1478, 1484, 1489, 1494, 
    1500, 1505, 1511, 1516, 1522, 1527, 1533, 1538, 1544, 1550, 
    1555, 1561, 1566, 1572, 1578, 1584, 1589, 1595, 1601, 1607, 
    1612, 1618, 1624, 1630, 1636, 1642, 1648, 1654, 1659, 1665, 
    1671, 1677, 1683, 1689, 1695, 1701, 1708, 1714, 1720, 1726, 
    1732, 1738, 1744, 1750, 1756, 1763, 1769, 1775, 1781, 1787, 
    1794, 1800, 1806, 1812, 1819, 1825, 1831, 1837, 1844, 1850, 
    1856, 1863, 1869, 1875, 1882, 1888, 1894, 1901, 1907, 1913, 
    1920, 1926, 1933, 1939, 1945, 1952, 1958, 1965, 1971, 1977, 
    1984, 1990, 1997, 2003, 2009, 2016, 2022, 2029, 2035, 2042, 
};