#define LOG_TAG         "Test"

#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"
#include "cts_test.h"
#include "cts_firmware.h"

bool test_debug = 0;
struct cts_log cts_log;
extern struct chipone_ts_data *g_cts_data;

/*static char *cts_test_config_data = \
                                                      "\n" "fw_version_test=1"
                                                      "\n" "short_test=1"
                                                      "\n"  "open_test=1"
                                                      "\n" "rawdata_test=1"
                                                      "\n"  "firmware_version=0x0900"
                                                      "\n" "short_threshold = 800"
                                                      "\n" "rawdata_max=3086"
                                                      "\n"  "rawdata_min=1841"
                                                        ;
                                                        */
#define SUPPORT_FW_VERSION_TEST		0
#define TARGET_FW_VERSION		0x1d01

#define SUPPORT_RAWDATA_TEST	1
#define SUPPORT_OPEN_TEST		1
#define SUPPORT_SHORT_TEST		1
#define SHORT_TEST_TH			800

u16 rawdata_test_min[] = {
	5337,  5752,  5710,  5658,  5639,  5578,  5596,  5547,  5577,  5517,  5505,  5464,  5494,  5047,  5221,  5147,  5189,  5158,  5176,  5132,  5160,  5116,  5167,  5089,  5115,  4877,  5569,  
	5374,  5443,  5395,  5341,  5316,  5248,  5254,  5207,  5232,  5175,  5152,  5117,  5133,  4858,  5022,  4961,  4998,  4967,  4985,  4951,  4979,  4942,  4985,  4919,  4943,  4715,  5014,  
	5404,  5458,  5403,  5341,  5316,  5245,  5247,  5195,  5222,  5161,  5139,  5099,  5117,  4894,  5064,  4998,  5038,  5004,  5027,  4986,  5023,  4983,  5029,  4960,  4994,  4757,  5017,  
	5437,  5472,  5409,  5345,  5320,  5246,  5251,  5198,  5224,  5169,  5142,  5099,  5115,  4932,  5101,  5036,  5074,  5041,  5060,  5021,  5056,  5022,  5064,  5001,  5033,  4800,  5043,  
	5407,  5430,  5358,  5293,  5270,  5195,  5196,  5149,  5173,  5113,  5086,  5048,  5059,  4911,  5088,  5021,  5059,  5023,  5043,  5007,  5041,  5003,  5051,  4990,  5024,  4792,  5015,  
	5444,  5467,  5385,  5323,  5290,  5222,  5217,  5179,  5197,  5138,  5108,  5076,  5080,  4973,  5150,  5090,  5121,  5089,  5105,  5075,  5105,  5074,  5117,  5061,  5093,  4867,  4583,  
	5442,  5478,  5393,  5343,  5304,  5252,  5238,  5211,  5222,  5174,  5131,  5116,  5108,  5031,  5222,  5177,  5194,  5180,  5183,  5174,  5179,  5173,  5206,  5169,  5181,  4946,  1235,  
	5263,  5294,  5221,  5166,  5133,  5075,  5077,  5038,  5048,  4988,  4962,  4933,  4935,  4913,  5073,  5016,  5050,  5021,  5033,  5008,  5031,  5008,  5044,  4996,  5026,  4821,  4462,  
	5225,  5261,  5194,  5139,  5105,  5044,  5046,  5007,  5017,  4962,  4934,  4905,  4905,  4917,  5078,  5028,  5060,  5034,  5044,  5018,  5050,  5021,  5057,  5009,  5036,  4828,  4945,  
	5177,  5220,  5155,  5100,  5071,  5009,  5013,  4968,  4983,  4928,  4911,  4873,  4874,  4923,  5094,  5036,  5075,  5045,  5060,  5028,  5068,  5032,  5076,  5025,  5057,  4841,  4937,  
	5131,  5193,  5137,  5085,  5061,  5003,  5006,  4960,  4971,  4920,  4905,  4868,  4869,  4954,  5129,  5072,  5110,  5078,  5094,  5062,  5103,  5068,  5113,  5060,  5093,  4873,  4941,  
	5088,  5161,  5110,  5064,  5042,  4989,  4989,  4950,  4962,  4913,  4894,  4863,  4859,  4989,  5163,  5110,  5144,  5118,  5135,  5108,  5145,  5109,  5153,  5104,  5136,  4917,  4957,  
	5010,  5444,  5396,  5342,  5319,  5258,  5259,  5205,  5229,  5174,  5152,  5111,  5112,  5468,  5625,  5554,  5591,  5556,  5583,  5548,  5597,  5549,  5605,  5546,  5587,  5342,  5335,  

};
  u16 rawdata_test_max[] = {
	10360,	11166,	11084,	10983,	10945,	10828,	10863,	10768,	10825,	10709,	10687,	10607,	10664,	9798,	10135,	9991,	10072,	10014,	10048,	9962,	10017,	9932,	10031,	9878,	9929,	9467,	10810, 
	10432,	10566,	10472,	10369,	10319,	10188,	10199,	10109,	10155,	10047,	10000,	9933,	9965,	9430,	9749,	9629,	9703,	9643,	9678,	9612,	9665,	9593,	9677,	9549,	9595,	9153,	9733,
	10490,	10594,	10488,	10368,	10320,	10181,	10185,	10083,	10136,	10018,	9976,	9898,	9932,	9500,	9830,	9702,	9780,	9714,	9758,	9679,	9750,	9673,	9762,	9628,	9694,	9234,	9738,
	10554,	10623,	10499,	10377,	10327,	10183,	10194,	10091,	10141,	10034,	9981,	9899,	9930,	9573,	9901,	9776,	9850,	9785,	9822,	9747,	9816,	9748,	9830,	9707,	9769,	9317,	9789,
	10495,	10541,	10401,	10276,	10230,	10083,	10086,	9995,	10042,	9925,	9873,	9799,	9820,	9534,	9876,	9746,	9821,	9751,	9789,	9719,	9786,	9713,	9805,	9686,	9752,	9302,	9735,
	10567,	10612,	10453,	10333,	10268,	10137,	10126,	10054,	10088,	9974,	9915,	9854,	9861,	9653,	9998,	9882,	9940,	9879,	9910,	9851,	9909,	9849,	9933,	9825,	9886,	9448,	8897,
	10564,	10634,	10468,	10371,	10297,	10196,	10167,	10115,	10136,	10043,	9960,	9931,	9916,	9765,	10137,	10049,	10082,	10055,	10061,	10045,	10054,	10042,	10107,	10033,	10057,	9602,	2397,
	10216,	10277,	10134,	10028,	9964,	9851,	9855,	9780,	9799,	9684,	9633,	9576,	9579,	9537,	9849,	9737,	9803,	9748,	9771,	9722,	9766,	9721,	9791,	9697,	9756,	9358,	8662,
	10143,	10213,	10083,	9977,	9910,	9792,	9795,	9719,	9739,	9632,	9578,	9521,	9522,	9546,	9858,	9759,	9822,	9772,	9792,	9742,	9802,	9748,	9816,	9724,	9776,	9373,	9599,
	10050,	10132,	10006,	9900,	9844,	9724,	9731,	9643,	9672,	9566,	9533,	9460,	9462,	9556,	9888,	9775,	9851,	9792,	9823,	9761,	9838,	9769,	9853,	9754,	9817,	9397,	9584,
	9960,	10081,	9971,	9870,	9824,	9712,	9717,	9628,	9650,	9552,	9521,	9450,	9452,	9617,	9957,	9845,	9919,	9857,	9888,	9826,	9906,	9837,	9926,	9822,	9885,	9460,	9592,
	9878,	10019,	9920,	9829,	9787,	9684,	9685,	9609,	9631,	9537,	9500,	9439,	9432,	9685,	10023,	9920,	9985,	9934,	9968,	9915,	9988,	9917,	10003,	9907,	9971,	9545,	9622,
	9725,	10567,	10475,	10371,	10326,	10207,	10208,	10104,	10149,	10045,	10001,	9921,	9923,	10613,	10919,	10781,	10853,	10785,	10839,	10769,	10864,	10771,	10881,	10765,	10846,	10369,	10356, 

};



 u16 open_rawdata_min[] = {
	1382,	1500,	1489,	1474,	1465,	1450,	1454,	1441,	1447,	1433,	1434,	1423,	1430,	1307,	1358,	1337,	1351,	1345,	1345,	1335,	1345,	1335,	1348,	1329,	1337,	1270,	1469, 
	1397,	1418,	1403,	1389,	1380,	1363,	1363,	1352,	1355,	1343,	1342,	1332,	1336,	1261,	1308,	1292,	1304,	1297,	1297,	1290,	1299,	1292,	1302,	1287,	1294,	1229,	1321, 
	1409,	1424,	1408,	1391,	1383,	1364,	1364,	1350,	1355,	1340,	1340,	1329,	1333,	1271,	1321,	1302,	1314,	1306,	1309,	1298,	1312,	1303,	1314,	1298,	1307,	1240,	1321, 
	1411,	1421,	1404,	1385,	1379,	1359,	1361,	1345,	1351,	1336,	1335,	1323,	1328,	1276,	1327,	1308,	1321,	1312,	1316,	1304,	1318,	1308,	1321,	1303,	1314,	1246,	1324, 
	1406,	1414,	1394,	1378,	1371,	1353,	1352,	1338,	1341,	1325,	1324,	1315,	1318,	1276,	1327,	1309,	1321,	1312,	1315,	1306,	1317,	1309,	1320,	1306,	1314,	1249,	1319, 
	1407,	1414,	1394,	1377,	1369,	1351,	1350,	1337,	1340,	1325,	1323,	1313,	1316,	1283,	1336,	1319,	1330,	1322,	1324,	1316,	1326,	1320,	1331,	1317,	1326,	1261,	1196, 
	1399,	1408,	1387,	1373,	1362,	1348,	1346,	1335,	1336,	1324,	1319,	1314,	1312,	1293,	1344,	1330,	1339,	1336,	1334,	1330,	1335,	1334,	1341,	1333,	1336,	1274,	362,
	1417,	1425,	1405,	1389,	1381,	1364,	1364,	1350,	1353,	1337,	1336,	1325,	1328,	1316,	1371,	1352,	1365,	1357,	1359,	1352,	1361,	1354,	1365,	1352,	1361,	1297,	1206, 
	1410,	1419,	1400,	1386,	1375,	1359,	1357,	1346,	1347,	1333,	1330,	1321,	1322,	1323,	1375,	1361,	1372,	1365,	1366,	1358,	1369,	1362,	1373,	1361,	1369,	1302,	1347, 
	1397,	1409,	1390,	1375,	1366,	1349,	1348,	1336,	1336,	1324,	1322,	1312,	1312,	1324,	1378,	1363,	1373,	1368,	1369,	1361,	1373,	1365,	1377,	1365,	1373,	1306,	1345, 
	1387,	1403,	1388,	1372,	1365,	1348,	1349,	1334,	1338,	1324,	1325,	1313,	1314,	1335,	1394,	1375,	1388,	1380,	1384,	1374,	1389,	1379,	1393,	1378,	1389,	1319,	1350, 
	1374,	1394,	1380,	1367,	1361,	1346,	1345,	1333,	1335,	1323,	1323,	1313,	1312,	1348,	1405,	1389,	1400,	1393,	1396,	1389,	1401,	1392,	1406,	1393,	1403,	1335,	1358, 
	1332,	1452,	1439,	1423,	1417,	1399,	1397,	1380,	1389,	1375,	1372,	1360,	1361,	1453,	1507,	1486,	1498,	1489,	1493,	1484,	1500,	1488,	1505,	1489,	1502,	1427,	1440, 


};		
 u16 open_rawdata_max[] = {
	2682,	2913,	2890,	2861,	2845,	2815,	2823,	2798,	2809,	2782,	2784,	2763,	2776,	2537,	2635,	2596,	2623,	2610,	2612,	2591,	2610,	2591,	2616,	2580,	2595,	2466,	2852, 
	2712,	2754,	2724,	2695,	2678,	2647,	2647,	2624,	2630,	2607,	2605,	2586,	2593,	2448,	2540,	2508,	2530,	2517,	2518,	2503,	2523,	2508,	2527,	2498,	2513,	2386,	2564, 
	2734,	2765,	2734,	2700,	2685,	2648,	2649,	2621,	2630,	2601,	2602,	2580,	2587,	2466,	2565,	2527,	2552,	2536,	2541,	2519,	2546,	2528,	2550,	2519,	2537,	2406,	2564, 
	2738,	2757,	2725,	2689,	2677,	2639,	2643,	2611,	2622,	2594,	2592,	2568,	2578,	2476,	2576,	2539,	2564,	2547,	2555,	2531,	2558,	2539,	2563,	2530,	2551,	2418,	2569, 
	2729,	2744,	2706,	2674,	2661,	2625,	2624,	2597,	2603,	2573,	2571,	2552,	2558,	2477,	2575,	2541,	2563,	2547,	2553,	2535,	2557,	2542,	2563,	2535,	2551,	2424,	2560, 
	2732,	2745,	2705,	2673,	2657,	2623,	2620,	2595,	2601,	2572,	2569,	2548,	2555,	2491,	2593,	2560,	2581,	2567,	2571,	2554,	2575,	2561,	2584,	2556,	2574,	2449,	2321, 
	2715,	2734,	2692,	2665,	2645,	2617,	2613,	2592,	2592,	2570,	2560,	2551,	2547,	2509,	2609,	2582,	2599,	2592,	2590,	2582,	2591,	2589,	2603,	2587,	2593,	2474,	703,
	2751,	2766,	2728,	2696,	2681,	2648,	2647,	2622,	2626,	2596,	2593,	2572,	2577,	2556,	2660,	2624,	2649,	2635,	2639,	2624,	2641,	2628,	2651,	2624,	2643,	2518,	2342, 
	2737,	2755,	2717,	2690,	2669,	2638,	2634,	2613,	2614,	2587,	2581,	2563,	2565,	2567,	2670,	2641,	2663,	2651,	2652,	2636,	2658,	2644,	2666,	2642,	2657,	2528,	2615, 
	2711,	2734,	2697,	2670,	2651,	2618,	2616,	2593,	2594,	2570,	2566,	2548,	2547,	2571,	2675,	2645,	2665,	2655,	2657,	2642,	2665,	2651,	2673,	2650,	2665,	2536,	2611, 
	2692,	2723,	2693,	2662,	2650,	2616,	2618,	2590,	2596,	2569,	2572,	2549,	2552,	2592,	2707,	2668,	2694,	2679,	2687,	2667,	2696,	2676,	2704,	2675,	2696,	2561,	2621, 
	2667,	2706,	2680,	2654,	2643,	2612,	2610,	2588,	2592,	2569,	2568,	2549,	2546,	2617,	2727,	2695,	2717,	2703,	2710,	2695,	2721,	2702,	2728,	2704,	2723,	2591,	2636, 
	2585,	2819,	2793,	2763,	2751,	2717,	2712,	2680,	2696,	2669,	2664,	2640,	2642,	2821,	2925,	2885,	2908,	2891,	2899,	2880,	2912,	2888,	2921,	2890,	2915,	2770,	2795, 

};

int cts_test_save_log(struct cts_device *cts_dev, const char *filepath, char *buf)
{

    struct file *file;
    //u32 size;
    u32 len;
    int ret;

#ifndef SUPPORT_SAVE_TEST_LOG
    cts_info("Unsupport save log file");
    return -1;
#endif

    if(buf == NULL){
        cts_err("buf is NULL");
        return -1;
    }
    
    file = filp_open(filepath, O_RDWR |O_CREAT |O_APPEND , 0666);//O_TRUNC  //O_APPEND
    if (IS_ERR(file)) {
        cts_err("Open file '%s' failed %ld", filepath, PTR_ERR(file));
        return -1;
    }

    len =strlen(buf);
    cts_info("write to file %s size: %d", filepath, len);
    ret = cts_file_write(file, buf, len, 0);
    if(ret != len){
        cts_err("kernel write %s fail %d",filepath,ret);
    }
    ret = filp_close(file, NULL);
    if (ret) {
        cts_warn("Close file '%s' failed %d", filepath, ret);
    }
    return ret;

}

int cts_fw_version_test_fun( struct cts_device * cts_dev, u16 para1, u16 para2)
{
    return cts_fw_version_test(cts_dev, para1);
}
int cts_short_test_fun( struct cts_device * cts_dev, u16 para1, u16 para2)
{   
    return cts_short_test(cts_dev, para1);
}

int cts_open_test_fun( struct cts_device * cts_dev, u16 *para1, u16 *para2)
{   
    return cts_open_test(cts_dev, para1, para2);
}

int cts_rawdata_test_fun( struct cts_device * cts_dev, u16 *para1, u16 *para2)
{   
    return cts_rawdata_test(cts_dev, para1, para2);
}

struct cts_test_cfg cts_test_items[] ={
    {
        .item_name = "fw_version_test",
		.need_test= SUPPORT_FW_VERSION_TEST,
        .result = 0,
        .para = {
            .para1 = "firmware_version",
        },
       // .cts_test_fun = cts_fw_version_test_fun,
    },
    
    {
        .item_name = "short_test",
		.need_test= SUPPORT_SHORT_TEST,
        .result = 0,
        .para = {
            .para1 = "short_threshold",
        },
       // .cts_test_fun = cts_short_test_fun,
    },
   
    {
        .item_name = "open_test",
		.need_test= SUPPORT_OPEN_TEST,
        .result = 0,
        //.para = {
        //    .para1 = "rawdata_min",
       // },
       // .cts_test_fun = cts_open_test_fun,
    },
    
    {
        .item_name = "rawdata_test",
		.need_test= SUPPORT_RAWDATA_TEST,
        .result = 0,
        .para = {
            .para1 = "rawdata_min",
            .para2 = "rawdata_max",
        },
       // .cts_test_fun = cts_rawdata_test_fun,
    },
};

int cts_tiny_short_test_init(struct cts_device *cts_dev)
{
    u32 regAddr[128];
    u8 regData[128];
    u32 regCount = 0;
    int ret,i;

    //Step1:  initial setting
    regAddr[regCount] = 0x040005; regData[regCount++] = 0x01;
    regAddr[regCount] = 0x040138; regData[regCount++] = 0x01;
    regAddr[regCount] = 0x040140; regData[regCount++] = 0x00;
    regAddr[regCount] = 0x04013c; regData[regCount++] = 0x00;
    regAddr[regCount] = 0x04010B; regData[regCount++] = 0x30;
    regAddr[regCount] = 0x040105; regData[regCount++] = 0x1a;
    regAddr[regCount] = 0x040136; regData[regCount++] = 0x30;
    regAddr[regCount] = 0x040100; regData[regCount++] = 0x07;
    regAddr[regCount] = 0x04011c; regData[regCount++] = 0xff;
    regAddr[regCount] = 0x04011d; regData[regCount++] = 0xff;
    regAddr[regCount] = 0x04011e; regData[regCount++] = 0x03;
    regAddr[regCount] = 0x04015c; regData[regCount++] = 0x00;
    regAddr[regCount] = 0x040195; regData[regCount++] = 0xF1;
    regAddr[regCount] = 0x0401B2; regData[regCount++] = 0x03;
    regAddr[regCount] = 0x0401BC; regData[regCount++] = 0x01;
    regAddr[regCount] = 0x040197; regData[regCount++] = 0x02;
    regAddr[regCount] = 0x040178; regData[regCount++] = 0x00;
    regAddr[regCount] = 0x04019C; regData[regCount++] = 0xFF;
    regAddr[regCount] = 0x0401B3; regData[regCount++] = 0x1F;
    regAddr[regCount] = 0x04088D; regData[regCount++] = 0x03;
    regAddr[regCount] = 0x04088C; regData[regCount++] = 0x84;
    regAddr[regCount] = 0x04088F; regData[regCount++] = 0x02;
    regAddr[regCount] = 0x04088E; regData[regCount++] = 0x58;
    regAddr[regCount] = 0x0408C4; regData[regCount++] = 0x20;
    regAddr[regCount] = 0x0408C5; regData[regCount++] = 0x00;
    regAddr[regCount] = 0x0408C6; regData[regCount++] = 0x01;
    regAddr[regCount] = 0x040841; regData[regCount++] = 0x0b;
    regAddr[regCount] = 0x040858; regData[regCount++] = 0x12;
    regAddr[regCount] = 0x040859; regData[regCount++] = 0x12;
    regAddr[regCount] = 0x040145; regData[regCount++] = 0x00;
    regAddr[regCount] = 0x040198; regData[regCount++] = 0x1A;
    regAddr[regCount] = 0x04010A; regData[regCount++] = 0x44;
    regAddr[regCount] = 0x04082C; regData[regCount++] = 0x08;
    for (i = 0; i < RX_CHANNEL_NUM / 2; i++)
    {
        regAddr[regCount] = (u32)(0x040800 + i); 
        regData[regCount++] = (u8)(0x40 + i * 2);
    }
    for (i = 0; i < RX_CHANNEL_NUM / 2; i++)
    {
        regAddr[regCount] = (u32)(0x040814 + i); 
        regData[regCount++] = (u8)(0x40 + i * 2 + 1);
    }
    regAddr[regCount] = 0x040874; regData[regCount++] = 0x01;
    regAddr[regCount] = 0x040868; regData[regCount++] = 0x01;
    regAddr[regCount] = 0x0408F0; regData[regCount++] = 0x01;

    //Debug.WriteLine("-----------InitialSetting-----------------");
    //for (int i = 0; i < regCount;i++ )
    //{
    //    Debug.WriteLine(string.Format("{0:X6}={1:X2}",regAddr[i],regData[i]));
    //}
    for(i=0; i< regCount; i++){
        ret = icn85xx_prog_i2c_txdata(cts_dev, regAddr[i], &regData[i], 1);
        if(ret)
            return 0;
        udelay(10);
    }

   // if (testConfig.chip.I2cWriteProgModeEx(regAddr, regData, regCount) != 0)
   // {
   //     return false;
   // }
    return 1;
}

int cts_get_adc_data(struct cts_device *cts_dev, u16 *adc_data, int len)
{
    ////////////////////////////////////////////
    u8 ucTemp[4];
    u8 data[128];
    u16 count = 0;
    int i;

    //adc stop
    ucTemp[0] = 0x01;
    if (icn85xx_prog_i2c_txdata(cts_dev, 0x0408ac, ucTemp, 1) != 0)
        return 0;

    ucTemp[0] = 0x0f;
    if (icn85xx_prog_i2c_txdata(cts_dev, 0x040100, ucTemp, 1) != 0)
        return 0;

    ucTemp[0] = 0x07;
    if (icn85xx_prog_i2c_txdata(cts_dev, 0x040100, ucTemp, 1) != 0)
        return 0;

    //adc start
    ucTemp[0] = 0x01;
    if (icn85xx_prog_i2c_txdata(cts_dev, 0x040870, ucTemp, 1) != 0)
        return 0;

    //Thread.Sleep(10);
    msleep(10);

    //poll status,Wait adc data ready
    //uint start = timeGetTime();
    while (1)
    {
        count++;
        if (icn85xx_prog_i2c_rxdata(cts_dev, 0x040870, ucTemp, 1) != 0)
            return 0;

        if ((ucTemp[0] & 0x02) != 0)
            break;

        msleep(1);
        if (count > 1000)
            return 0;
    }

    //adc stop
    ucTemp[0] = 0x01;
    if (icn85xx_prog_i2c_txdata(cts_dev, 0x0408ac, ucTemp, 1) != 0)
        return 0;

    //Read ADC data from SRAM 0x24000.
    if (icn85xx_prog_i2c_rxdata(cts_dev, 0x24000, data, (u32)len * 2) != 0)
        return 0;


    for(i=0;i<len;i++)
    {
        adc_data[i] = (u16)(data[i * 2] + data[i * 2 + 1] * 256);
    }
    return 1;
    
}

 int cts_get_short_resistor_data(struct cts_device *cts_dev, u16 *data, enum adc_data type)
{
     u32 regAddr[128];
     u16 temp_data[RX_CHANNEL_NUM];
     u8 regData[128];
     u8 ucTemp[4] = {0};
     u32 regCount = 0;
     int ret,i;

     switch (type)
     {
         case CHANNEL_OFFSET:
             regCount = 0;
             regAddr[regCount] = 0x040118; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x040119; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x040136; regData[regCount++] = 0x30;
             regAddr[regCount] = 0x0401A0; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x0401A1; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x0401A2; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x0401A3; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x0401A4; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x0401C0; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x0401C1; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x0401C2; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x0401C3; regData[regCount++] = 0x00;

             //Debug.WriteLine(string.Format("-----------{0}-----------------", type.ToString()));
             //for (int i = 0; i < regCount; i++)
             //{
             //    Debug.WriteLine(string.Format("{0:X6}={1:X2}", regAddr[i], regData[i]));
             //}

            // if (icn85xx_prog_i2c_txdata(cts_dev, regAddr, regData, regCount) != 0)
            //     return 0;
             for(i=0; i< regCount; i++){
                ret = icn85xx_prog_i2c_txdata(cts_dev, regAddr[i], &regData[i], 1);
                if(ret)
                    return 0;
                udelay(10);
            }

             if (cts_get_adc_data(cts_dev, data, RX_CHANNEL_NUM) == 0)
                 return 0;

             break;
         case INTERNAL_REFERENCE_RESISTOR:
             regCount = 0;
             regAddr[regCount] = 0x040118; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x040119; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x040136; regData[regCount++] = 0x31;
             regAddr[regCount] = 0x0401A0; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x0401A1; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x0401A2; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x0401A3; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x0401A4; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x0401C0; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x0401C1; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x0401C2; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x0401C3; regData[regCount++] = 0x00;

             //Debug.WriteLine(string.Format("-----------{0}-----------------", type.ToString()));
             //for (int i = 0; i < regCount; i++)
             //{
             //    Debug.WriteLine(string.Format("{0:X6}={1:X2}", regAddr[i], regData[i]));
             //}

            // if (testConfig.chip.I2cWriteProgModeEx(regAddr, regData, regCount) != 0)
            //     return false;
             for(i=0; i< regCount; i++){
                ret = icn85xx_prog_i2c_txdata(cts_dev, regAddr[i], &regData[i], 1);
                if(ret)
                    return 0;
                udelay(10);
            }

             if (cts_get_adc_data(cts_dev, data, RX_CHANNEL_NUM) == 0)
                 return 0;

             break;
         case ODD_RX_TO_GROUND_SHORT_RESISTOR:
             regCount = 0;
             regAddr[regCount] = 0x040118; regData[regCount++] = 0x10;
             regAddr[regCount] = 0x040119; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x040136; regData[regCount++] = 0x30;
             regAddr[regCount] = 0x0401A0; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401A1; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401A2; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401A3; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401A4; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401C0; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401C1; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401C2; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401C3; regData[regCount++] = 0xff;

             //Debug.WriteLine(string.Format("-----------{0}-----------------", type.ToString()));
             //for (int i = 0; i < regCount; i++)
             //{
             //    Debug.WriteLine(string.Format("{0:X6}={1:X2}", regAddr[i], regData[i]));
             //}

             //if (testConfig.chip.I2cWriteProgModeEx(regAddr, regData, regCount) != 0)
             //    return false;
             for(i=0; i< regCount; i++){
                ret = icn85xx_prog_i2c_txdata(cts_dev, regAddr[i], &regData[i], 1);
                if(ret)
                    return 0;
                udelay(10);
             }

             for (i = 0; i < RX_CHANNEL_NUM / 2; i++)
             {
                 //byte[] ucTemp = new byte[4];
                 //Array.Clear(ucTemp, 0, ucTemp.Count());
                 memset(ucTemp,0x00,sizeof(ucTemp));
                 ucTemp[i / 8] = (u8)(1 << (i % 8));
                 if (icn85xx_prog_i2c_txdata(cts_dev, 0x04011c, ucTemp, 3) != 0)
                     return 0;

                 //for (int j = 0; j < 3; j++)
                 //{
                 //    Debug.WriteLine(string.Format("{0:X6}={1:X2}", 0x04011c + j, ucTemp[j]));
                 //}

                 //UInt16[] temp_data = new UInt16[RX_CHANNEL_NUM];
                 if (cts_get_adc_data(cts_dev, temp_data, RX_CHANNEL_NUM) == 0)
                     return 0;

                 data[i * 2] = temp_data[i * 2];
             }

             break;
         case EVEN_RX_TO_GROUND_SHORT_RESISTOR:
             regAddr[regCount] = 0x040118; regData[regCount++] = 0x20;
             regAddr[regCount] = 0x040119; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x040136; regData[regCount++] = 0x30;
             regAddr[regCount] = 0x0401A0; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401A1; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401A2; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401A3; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401A4; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401C0; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401C1; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401C2; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401C3; regData[regCount++] = 0xff;

             //Debug.WriteLine(string.Format("-----------{0}-----------------", type.ToString()));
             //for (int i = 0; i < regCount; i++)
             //{
             //    Debug.WriteLine(string.Format("{0:X6}={1:X2}", regAddr[i], regData[i]));
             //}

             //if (testConfig.chip.I2cWriteProgModeEx(regAddr, regData, regCount) != 0)
             //    return false;
             for(i=0; i< regCount; i++){
                ret = icn85xx_prog_i2c_txdata(cts_dev, regAddr[i], &regData[i], 1);
                if(ret)
                    return 0;
                udelay(10);
             }

             for (i = 0; i < RX_CHANNEL_NUM / 2; i++)
             {
                 //byte[] ucTemp = new byte[4];
                 //Array.Clear(ucTemp, 0, ucTemp.Count());
                memset(ucTemp,0x00,sizeof(ucTemp));

                 ucTemp[i / 8] = (u8)(1 << (i % 8));
                 if (icn85xx_prog_i2c_txdata(cts_dev, 0x04011c, ucTemp, 3) != 0)
                     return 0;

                 //for (int j = 0; j < 3; j++)
                 //{
                 //    Debug.WriteLine(string.Format("{0:X6}={1:X2}", 0x04011c + j, ucTemp[j]));
                 //}

                 //UInt16[] temp_data = new UInt16[RX_CHANNEL_NUM];
                 if (cts_get_adc_data(cts_dev, temp_data, RX_CHANNEL_NUM) == 0)
                     return 0;

                 data[i * 2] = temp_data[i * 2];
             }

             break;
         case ODD_TX_TO_GROUND_SHORT_RESISTOR:
             regAddr[regCount] = 0x040118; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x040119; regData[regCount++] = 0x01;
             regAddr[regCount] = 0x040136; regData[regCount++] = 0x30;
             regAddr[regCount] = 0x0401A0; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401A1; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401A2; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401A3; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401A4; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401C0; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401C1; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401C2; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401C3; regData[regCount++] = 0xff;

             //if (testConfig.chip.I2cWriteProgModeEx(regAddr, regData, regCount) != 0)
             //    return false;
             for(i=0; i< regCount; i++){
                ret = icn85xx_prog_i2c_txdata(cts_dev, regAddr[i], &regData[i], 1);
                if(ret)
                    return 0;
                udelay(10);
             }

             for (i = 0; i < TX_CHANNEL_NUM / 2; i++)
             {
                 //byte[] ucTemp = new byte[4];
                 //Array.Clear(ucTemp, 0, ucTemp.Count());
                memset(ucTemp,0x00,sizeof(ucTemp));
                 ucTemp[i / 8] = (u8)(1 << (i % 8));
                 if (icn85xx_prog_i2c_txdata(cts_dev, 0x04011c, ucTemp, 3) != 0)
                     return 0;

                 //UInt16[] temp_data = new UInt16[TX_CHANNEL_NUM];
                 if (cts_get_adc_data(cts_dev,  temp_data, TX_CHANNEL_NUM) == 0)
                     return 0;

                 data[i * 2] = temp_data[i * 2];
             }

             break;
         case EVEN_TX_TO_GROUND_SHORT_RESISTOR:
             regAddr[regCount] = 0x040118; regData[regCount++] = 0x00;
             regAddr[regCount] = 0x040119; regData[regCount++] = 0x02;
             regAddr[regCount] = 0x040136; regData[regCount++] = 0x30;
             regAddr[regCount] = 0x0401A0; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401A1; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401A2; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401A3; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401A4; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401C0; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401C1; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401C2; regData[regCount++] = 0xff;
             regAddr[regCount] = 0x0401C3; regData[regCount++] = 0xff;

            // if (testConfig.chip.I2cWriteProgModeEx(regAddr, regData, regCount) != 0)
             //    return false;
             for(i=0; i< regCount; i++){
                ret = icn85xx_prog_i2c_txdata(cts_dev, regAddr[i], &regData[i], 1);
                if(ret)
                    return 0;
                udelay(10);
             }

             for (i = 0; i < TX_CHANNEL_NUM / 2; i++)
             {
                // byte[] ucTemp = new byte[4];
               //  Array.Clear(ucTemp, 0, ucTemp.Count());
                 memset(ucTemp,0x00,sizeof(ucTemp));
                 ucTemp[i / 8] = (u8)(1 << (i % 8));
                 if (icn85xx_prog_i2c_txdata(cts_dev, 0x04011c, ucTemp, 3) != 0)
                     return 0;

                // UInt16[] temp_data = new UInt16[TX_CHANNEL_NUM];
                 if (cts_get_adc_data(cts_dev,  temp_data, TX_CHANNEL_NUM) == 0)
                     return 0;

                 data[i * 2] = temp_data[i * 2];
             }

             break;
     }
     return 1;
 }  

 int cts_get_tiny_short_data(struct cts_device *cts_dev, 
    struct tiny_short_config *tiny_short_config)
{
    u16 temp_data[RX_CHANNEL_NUM];
    u16 offset_data[RX_CHANNEL_NUM];
    u16 ref_data[RX_CHANNEL_NUM];
    u16 rx_data[RX_CHANNEL_NUM];
    
    u16 tx_data[TX_CHANNEL_NUM];
    //float rx_registor[RX_CHANNEL_NUM];
    //float tx_registor[TX_CHANNEL_NUM];
   // float a;
    //float b;    
    int a,b;
    int rx_registor[RX_CHANNEL_NUM];
    int tx_registor[TX_CHANNEL_NUM];
    int i;        



    if (cts_tiny_short_test_init(cts_dev) == false)
        return false;

    if (cts_get_short_resistor_data(cts_dev, temp_data, CHANNEL_OFFSET) == false)
        return false;

    for (i = 0; i < RX_CHANNEL_NUM; i += 2)
    {
        offset_data[i] = temp_data[i];
        offset_data[i+1] = temp_data[i];
    }

    if (cts_show_debug_log)
    {
        cts_info("offset_data");
        for (i = 0; i < RX_CHANNEL_NUM; i++)
            cts_info("%d",offset_data[i]);
    }

    //////////////////////////REF DATA
    if (cts_get_short_resistor_data(cts_dev, temp_data, INTERNAL_REFERENCE_RESISTOR) == 0)
        return 0;

    for (i = 0; i < RX_CHANNEL_NUM; i += 2)
    {
        ref_data[i] = temp_data[i];
        ref_data[i + 1] = temp_data[i];
    }

    if (test_debug)
    {
        cts_info("ref_data");
        for (i = 0; i < RX_CHANNEL_NUM; i++)
            cts_info("%d",ref_data[i]);
    }      

    //////////////////////////RX DATA
    if (cts_get_short_resistor_data(cts_dev, temp_data, ODD_RX_TO_GROUND_SHORT_RESISTOR) == 0)
        return 0;
    for (i = 0; i < RX_CHANNEL_NUM; i+=2)
    {
        rx_data[i] = temp_data[i];
    }
    if (cts_get_short_resistor_data(cts_dev, temp_data, EVEN_RX_TO_GROUND_SHORT_RESISTOR) == 0)
        return 0;
    for ( i = 0; i < RX_CHANNEL_NUM; i += 2)
    {
        rx_data[i + 1] = temp_data[i];
    }

    if (test_debug)
    {
        cts_info("rx_data");
        for (i = 0; i < RX_CHANNEL_NUM; i++)
            cts_info("%d",rx_data[i]);
    }

    //////////////////////////TX DATA
    if (cts_get_short_resistor_data(cts_dev, temp_data, ODD_TX_TO_GROUND_SHORT_RESISTOR) == 0)
        return 0;
    for (i = 0; i < TX_CHANNEL_NUM; i += 2)
    {
        tx_data[i] = temp_data[i];
    }
    if (cts_get_short_resistor_data(cts_dev, temp_data, EVEN_TX_TO_GROUND_SHORT_RESISTOR) == 0)
        return 0;
    for (i = 0; i < TX_CHANNEL_NUM; i += 2)
    {
        tx_data[i + 1] = temp_data[i];
    }

    if (test_debug)
    {
        cts_info("tx_data");
        for (i = 0; i < TX_CHANNEL_NUM; i++)
            cts_info("%d",tx_data[i]);
    }

    for (i = 0; i < RX_CHANNEL_NUM; i++)
    {
        //a = (float)fabs((float)ref_data[i] - (float)offset_data[i]);
        //b = (float)fabs((float)rx_data[i] - (float)offset_data[i]);
        a = (int)((ref_data[i] >= offset_data[i])? (ref_data[i]-offset_data[i]):(offset_data[i]-ref_data[i]));
        b = (int)((rx_data[i] >= offset_data[i])? (rx_data[i]-offset_data[i]):(offset_data[i]-rx_data[i]));
        if( b < 1)
            b = 1;
        //rx_registor[i] = (a / b * 2 - 1) * 100;
        rx_registor[i] = (a*100 / b * 2 - 100);
        if(test_debug)
            cts_info("a:%d, b:%d, rx_registor[%d]:%d", a, b, i, rx_registor[i]);
        tiny_short_config->rx_data[i] = (u32)rx_registor[i];
    }

    if (test_debug)
    {
        for ( i = 0; i < RX_CHANNEL_NUM; i++)
        {
            cts_info("RX Resistor: %d ,", tiny_short_config->rx_data[i]);
        }
    }

    for (i = 0; i < TX_CHANNEL_NUM; i++)
    {
        //float a = (float)fabs((float)ref_data[i] - (float)offset_data[i]);
        //float b = (float)fabs((float)tx_data[i] - (float)offset_data[i]);
        a = (int)((ref_data[i] >= offset_data[i])? (ref_data[i]-offset_data[i]):(offset_data[i]-ref_data[i]));
        b = (int)((tx_data[i] >= offset_data[i])? (tx_data[i]-offset_data[i]):(offset_data[i]-tx_data[i]));
        if (b < 1)
            b = 1;
        //tx_registor[i] = (a / b * 2 - 1) * 100;
        tx_registor[i] = (a*100 / b * 2 - 100) ;
        if(test_debug)
            cts_info("a:%d,b:%d,tx_registor[%d]:%d", a, b, i, tx_registor[i]);
        tiny_short_config->tx_data[i] = (u32)tx_registor[i];
    }

    if (test_debug)
    {
        for ( i = 0; i < TX_CHANNEL_NUM; i++)
        {
            cts_info("TX Resistor: %d ,", tiny_short_config->tx_data[i]);
        }
    }

    return 1;
}

int cts_start_tiny_short_test(struct cts_device * cts_dev, char *buf,
        struct tiny_short_config *tiny_short_config)
{
    u8 table[4] = { 3, 2, 1, 0 };
    int index;
    int i;
    u32 th = (u32)(tiny_short_config->tiny_short_threshold);
    
    tiny_short_config->tiny_short_result = true;

    for (i = 0; i < RX_CHANNEL_NUM; i++)
    {
        tiny_short_config->rx_status[i] = 0;
    }
    for (i = 0; i < TX_CHANNEL_NUM; i++)
    {
        tiny_short_config->tx_status[i] = 0;
    }

    if (cts_get_tiny_short_data(cts_dev,tiny_short_config) == 1)
    {
        tiny_short_config->tiny_short_resistor = 0;
        tiny_short_config->short_tx_num = 0;
        tiny_short_config->short_rx_num = 0;
        
        for (i = 0; i < RX_CHANNEL_NUM; i++)
        {
            if (tiny_short_config->rx_data[i] < th)
            {
                tiny_short_config->tiny_short_result  = false;
                tiny_short_config->rx_status[i] = 1;
                if (tiny_short_config->rx_data[i] > tiny_short_config->tiny_short_resistor){
                    tiny_short_config->tiny_short_resistor = (int)tiny_short_config->rx_data[i];
                }
                cts_err("RX  physical index: %d tiny short test fail resistor: %d!!!", i, (int)tiny_short_config->rx_data[i]);
            }
        }

        for (i = 0; i < TX_CHANNEL_NUM; i++)
        {
            if (tiny_short_config->tx_data[i] < th)
            {
                tiny_short_config->tiny_short_result  = false;
                tiny_short_config->tx_status[i] = 1;
                if (tiny_short_config->tx_data[i] > tiny_short_config->tiny_short_resistor){
                    tiny_short_config->tiny_short_resistor = (int)tiny_short_config->tx_data[i];
                }
                cts_err("TX  physical index: %d tiny short test fail resistor: %d!!!",i, (int)tiny_short_config->tx_data[i]);

            }
        }

        for (i = 0; i < RX_CHANNEL_NUM; i++)
        {
            tiny_short_config->rx_logic_data[i] = 0;
            tiny_short_config->rx_logic_status[i] = 0;
        }

        for (i = 0; i < TX_CHANNEL_NUM; i++)
        {
            tiny_short_config->tx_logic_data[i] = 0;
            tiny_short_config->tx_logic_status[i] = 0;
        }

        cts_log.log_cnt += sprintf(buf+cts_log.log_cnt, "short threshold:%d\n",th);
        // rx data
        cts_log.log_cnt += sprintf(buf+cts_log.log_cnt, "rx_logic_data:");
        
        for ( i = 0; i < tiny_short_config->u8col_num; i++)
        {
            tiny_short_config->rx_logic_data[i] = tiny_short_config->rx_data[tiny_short_config->rx_order[i]];
            tiny_short_config->rx_logic_status[i] = tiny_short_config->rx_status[tiny_short_config->rx_order[i]];
            if(tiny_short_config->rx_logic_status[i]){
                tiny_short_config->short_rx_num++;
                cts_err("Logic order: RX%d tiny short test fail resistor: %d !!!", i+1, tiny_short_config->rx_logic_data[i]);
            }
            
            if((i%10) == 0){
                cts_log.log_cnt += sprintf(buf+cts_log.log_cnt, "\n");
            }
            cts_log.log_cnt += sprintf(buf+cts_log.log_cnt, "%-8d", tiny_short_config->rx_logic_data[i]);
        }

        for ( i = 0; i < tiny_short_config->u8col_num; i++){
            if(tiny_short_config->rx_logic_status[i]){
                cts_log.log_cnt += sprintf(buf+cts_log.log_cnt, "\nLogic order: RX%d tiny short test fail resistor: %d !!!\n",
                    i+1, tiny_short_config->rx_logic_data[i]);
            }
        }
        // tx data
        cts_log.log_cnt += sprintf(buf+cts_log.log_cnt, "\ntx_logic_data:");
        
        for ( i = 0; i < tiny_short_config->u8row_num; i++)
        {
            if (tiny_short_config->tx_order[i] > TX_CHANNEL_NUM - 2)
            {
                table[0] = 3;
                table[1] = 2;
                table[2] = 1;
                table[3] = 0;
                index = (RX_CHANNEL_NUM - 4) + table[(tiny_short_config->tx_order[i] - (TX_CHANNEL_NUM - 1))];
                tiny_short_config->tx_logic_data[i] = tiny_short_config->rx_data[index];
                tiny_short_config->tx_logic_status[i] = tiny_short_config->rx_status[index];
            }
            else
            {
                tiny_short_config->tx_logic_data[i] = tiny_short_config->tx_data[tiny_short_config->tx_order[i]];
                tiny_short_config->tx_logic_status[i] = tiny_short_config->tx_status[tiny_short_config->tx_order[i]];
            }
            if(tiny_short_config->tx_logic_status[i]){
                tiny_short_config->short_tx_num++;
                cts_err("Logic order: TX%d tiny short test fail resistor: %d !!!", i+1, tiny_short_config->tx_logic_data[i]);
            }
            
            if((i%10) == 0){
                cts_log.log_cnt += sprintf(buf+cts_log.log_cnt, "\n");
            }
            cts_log.log_cnt += sprintf(buf+cts_log.log_cnt, "%-8d", tiny_short_config->tx_logic_data[i]);

        }

        for ( i = 0; i < tiny_short_config->u8row_num; i++){
            if(tiny_short_config->tx_logic_status[i]){
                cts_log.log_cnt += sprintf(buf+cts_log.log_cnt, "\nLogic order: TX%d tiny short test fail resistor: %d !!!",
                    i+1, tiny_short_config->tx_logic_data[i]);
            }
        }
        cts_log.log_cnt += sprintf(buf+cts_log.log_cnt, "\n");

    }else{
        tiny_short_config->tiny_short_result = false;
        return -1;
    }

    return 0;
}

int cts_rawdata_alu(struct cts_device *cts_dev, char *buf, s16 *rawdata, u16 *threshold_min, 
        u16 *threshold_max, u8 is_open_test)
{
#define RAWDATA_BUFFER_SIZE(cts_dev) \
    (cts_dev->fwdata.rows * cts_dev->fwdata.cols * 2)

    s16 *result = NULL;
    int i,j,ret=0;
    u16 ng_num = 0;
   // u16 *th_min, *th_max;
    u16 count = 0;

    cts_info("rawdata process");
    result = (s16 *)kmalloc(RAWDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (result == NULL) {
        cts_err("Allocate memory for rawdata failed");
        return -1;
    }
#if 0
    if(is_open_test){
        th_min = threshold_min;
        th_max = 0xffff;
    }else{
        th_min = threshold_min;
        th_max = threshold_max;
    }
#endif
    cts_info("th_min:%d, th_max: %d",threshold_min[0],threshold_max[0]);
    count += sprintf(buf+count, "th_min:%d, th_max: %d\n",threshold_min[0],threshold_max[0]);
    for(i=0; i<cts_dev->fwdata.rows; i++){
        for(j=0; j<cts_dev->fwdata.cols; j++){
            if((rawdata[i*cts_dev->fwdata.cols + j] < threshold_min[i*cts_dev->fwdata.cols + j])
                ||(rawdata[i*cts_dev->fwdata.cols + j] > threshold_max[i*cts_dev->fwdata.cols + j])){
                result[i*cts_dev->fwdata.cols + j] = 1;
                ng_num++;
                cts_err("Test node:[TX%d,RX%d]:%d FAIL",i+1,j+1,rawdata[i*cts_dev->fwdata.cols + j]);
            }
            count += sprintf(buf+count, "%-6d", rawdata[i*cts_dev->fwdata.cols + j]);
        }
        count += sprintf(buf+count, "\n");
    }
    
    //open test: result save as format: tx[rows]+rx[cols]
    if(is_open_test && ng_num){
        // tx open test...
        cts_info("TX open test");
        count += sprintf(buf+count, "TX open test...\n");
        for(i=0; i<cts_dev->fwdata.rows; i++){
            result[i] = 0;
            for(j=0; j<cts_dev->fwdata.cols; j++){
                if((rawdata[i*cts_dev->fwdata.cols + j] < threshold_min[i*cts_dev->fwdata.cols + j])
                    ||(rawdata[i*cts_dev->fwdata.cols + j] > threshold_max[i*cts_dev->fwdata.cols + j])){
                    result[i]++;
                }
                if(j==cts_dev->fwdata.cols-1){
                    if(result[i] == cts_dev->fwdata.cols){
                        result[i] = 1;
                        ret++;
                        cts_err("TX%d open test fail",i+1);
                        count += sprintf(buf+count, "TX%d open test fail\n",i+1);
                    }else{
                        result[i] = 0;
                    }
                }
            }
        }
        // rx open test...
        cts_info("RX open test");
        count += sprintf(buf+count, "RX open test...\n");
        for(j=0; j<cts_dev->fwdata.cols; j++){
            result[cts_dev->fwdata.rows + j] = 0;
            for(i=0; i<cts_dev->fwdata.rows; i++){
                if((rawdata[i*cts_dev->fwdata.cols + j] < threshold_min[i*cts_dev->fwdata.cols + j])
                    ||(rawdata[i*cts_dev->fwdata.cols + j] > threshold_max[i*cts_dev->fwdata.cols + j])){
                    result[cts_dev->fwdata.rows + j]++;
                }
                if(i==cts_dev->fwdata.rows-1){
                    if(result[cts_dev->fwdata.rows + j] == cts_dev->fwdata.rows){
                        result[cts_dev->fwdata.rows + j] = 1;
                        ret++;
                        cts_err("RX%d open test fail",j+1);
                        count += sprintf(buf+count, "RX%d open test fail\n",j+1);
                    }else{
                        result[cts_dev->fwdata.rows + j] = 0;
                    }
                }
            }
        }
    }else{
        //rawdata test
        ret =  ng_num;
    }
    cts_log.log_cnt += count;
    
    kfree(result);

    return ret;
#undef RAWDATA_BUFFER_SIZE
}
int cts_open_prepare(struct cts_device *cts_dev)
{
	u8 cmd;
	int i;
	int ret;
	
	cts_info("open prepare");

	//scan mode
	cmd = 0;
	ret = cts_fw_reg_writeb_retry(cts_dev, CTS_DEVICE_FW_REG_SCAN_MODE, cmd, 2, 1);
	if (ret) {
            cts_err("set scan mode failed %d", ret);
     }
	
	cmd = 0x01;
	ret = cts_fw_reg_writeb_retry(cts_dev, CTS_DEVICE_FW_REG_WORK_MODE, cmd, 2, 1);
	if (ret) {
            cts_err("set factory mode failed %d", ret);
     }
	
	for (i = 0; i < 50; i++) {
        u8 ready;
        mdelay(10);
        ret = cts_fw_reg_readb_retry(cts_dev,
                    CTS_DEVICE_FW_REG_SYSTEM_BUSY, &ready, 3, 1);
        if (ret) {
            cts_err("Get system busy flag failed %d", ret);
            return ret;
        }
        if (ready == 0) 
            break;
        else
			cts_info("system busy flag= %d", ready);
    }

	if(i >= 400)
		return -ENODEV;
	
	cts_info("goto factory mode success");
	return ret;
}

/* Return 0 success 
          negative value while error occurs
          positive value means how many nodes fail */
int cts_rawdata_process(struct cts_device *cts_dev,  char *buf,
        u16 *threshold_min, u16 *threshold_max, u8 is_open_test)
{
#define RAWDATA_BUFFER_SIZE(cts_dev) \
    (cts_dev->fwdata.rows * cts_dev->fwdata.cols * 2)
    
    s16 *rawdata = NULL;
    //u8 row_index[128]={0};
    //u8 col_index[128]={0};

    int  ret;

    cts_info("cts rawdata process");

    rawdata = (s16 *)kmalloc(RAWDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (rawdata == NULL) {
        cts_err("Allocate memory for rawdata failed");
        return -ENOMEM;
    }
    cts_stop_device(cts_dev);
    cts_dev->rtdata.testing = true;

    ret = cts_enable_get_rawdata(cts_dev);
    if (ret) {
        cts_err("Enable read raw data failed %d", ret);
        goto err_free_rawdata;
    }

    ret = cts_send_command(cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
    if (ret) {
        cts_err("Send cmd QUIT_GESTURE_MONITOR failed %d", ret);
        goto err_free_rawdata;
    }
    msleep(50);

    ret = cts_get_rawdata(cts_dev, rawdata);
    if(ret) {
        cts_err("Get raw data failed %d", ret);
        // Fall through to disable get rawdata
    }
    ret = cts_disable_get_rawdata(cts_dev);
    if (ret) {
        cts_err("Disable read raw data failed %d", ret);
        // Fall through to show rawdata
    }

    ret = cts_rawdata_alu(cts_dev, buf, rawdata, threshold_min, threshold_max, is_open_test);
    
 err_free_rawdata:
    kfree(rawdata);
    cts_dev->rtdata.testing = false;
    cts_start_device(cts_dev);

    return ret;
    
#undef RAWDATA_BUFFER_SIZE

}


int cts_parse_test_para(char *test_config,  char *name, u16 *th)
{
    char *item_name;
    char *item_value;
    char *temp_ptr;
    char temp[128];
    u16 value;
    int count = 0;
    u16 len;
    int ret;
    
    item_name = strstr(test_config, name);
    if(item_name){
        len = strlen(name);
        cts_info("parse para name: %s len: %d",name,len);
        if(len > sizeof(temp)){
            cts_info("parse para name too long!!! len: %d",len);
            return false;
        }
        
        item_value = strchr(item_name, '=');//item_name + len +1; 
        if(item_value == NULL){
            cts_err("parse para value not found");
            return false;
        }
        
        item_value += 1;
        count = 0;
        while(item_value[count] !='\r' 
                    && item_value[count] !='\n'
                    && item_value[count] !='\0'){
            count++;
        }
        if(count >= sizeof(temp)){
            cts_info("parse para value too long!!! len: %d",count);
            return false;
        }

        temp_ptr = strncpy((char*)&temp[0], item_value, count);
        temp_ptr[count] = '\0';
        temp_ptr = strim(temp);
        
        cts_info("parse para value :%s count: %d",temp_ptr,count);
        
        if(isdigit(temp_ptr[0])){
            ret = kstrtou16(temp_ptr, 0, &value);
            cts_info("parse para:%s = %d [0x%x]", name, value, value);
            if(ret){
                cts_err("parse para value error!!!");
                return false;
            }
            *th = value;
            return true;
        }else{
            cts_err("parse para value invalid:%c",temp_ptr[0]);
        }
    }else{
        cts_err("parse para : %s not found in config file", name);
    }

    return false;
}

int cts_fw_version_test( struct cts_device * cts_dev, u16 fw_ver)
{
    u16 version;
    int ret;
    char *buf = NULL;
    
    cts_info("cts firmware version test");
    
   // cts_info("request log memery size: %d", PAGE_SIZE);
    buf = (char*)kzalloc(PAGE_SIZE, GFP_KERNEL);
    if(buf == NULL){
        cts_err("allocate memery for firmware version test log fail");
        return -ENOMEM;
    }
    cts_log.log_cnt = 0;
    cts_log.log_cnt += sprintf(buf, "\nStart firmware version test...\n"); 

    ret = cts_get_firmware_version(cts_dev, &version);
    if(ret){
        cts_err("cts firmware version test error");
        cts_log.log_cnt += sprintf(buf + cts_log.log_cnt, "cts firmware version test error\n"); 
        cts_test_save_log(cts_dev, CTS_TEST_LOG_PATH, buf);
        kfree(buf);
        return ret;
    }
    if(fw_ver ==  version){
        cts_info("cts firmware version: 0x%x test PASS",version);
        cts_log.log_cnt += sprintf(buf + cts_log.log_cnt, "cts firmware version: 0x%x test PASS\n",version); 
        cts_test_save_log(cts_dev, CTS_TEST_LOG_PATH, buf);
        kfree(buf);
        return 0;
    }else{
        cts_info("cts firmware version test FAIL");
        cts_log.log_cnt += sprintf(buf + cts_log.log_cnt,
                                        "cts firmware current version: 0x%x!= 0x%x test FAIL\n",version,fw_ver); 
        cts_test_save_log(cts_dev, CTS_TEST_LOG_PATH, buf);
        kfree(buf);
        return -1;
    }
}
 
/* Return 0 success 
          negative value while error occurs
          positive value means how many nodes fail */
int cts_short_test(struct cts_device *cts_dev, u16 threshold)
{
    int  ret;
    int  err_num = 0;
    char *buf = NULL;
    struct tiny_short_config *tiny_short_config = NULL;
    
    cts_info("cts short test");
    cts_stop_device(cts_dev);

    cts_dev->rtdata.testing = true;
    tiny_short_config = (struct tiny_short_config *)kzalloc(sizeof(*tiny_short_config), GFP_KERNEL);
    if (tiny_short_config == NULL) {
        cts_err("Allocate tiny_short_config failed");
        return -ENOMEM;
    }
    
   // cts_info("request short test log memery size: %d", PAGE_SIZE);
    buf = (char*)kzalloc(PAGE_SIZE, GFP_KERNEL);
    if(buf == NULL){
        cts_err("allocate memery for short test log fail");
        return -ENOMEM;
    }
    
    tiny_short_config->u8row_num = cts_dev->fwdata.rows;
    tiny_short_config->u8col_num = cts_dev->fwdata.cols;
    tiny_short_config->tiny_short_threshold = threshold;
    tiny_short_config->tiny_short_result = false;
    
    ret = cts_get_para_rx_order(cts_dev, tiny_short_config->rx_order);
    if(ret){
        cts_err("get para rx order fail");
    }
    ret = cts_get_para_tx_order(cts_dev, tiny_short_config->tx_order);
    if(ret){
        cts_err("get para tx order fail");
    }

    cts_enter_program_mode(cts_dev);

    cts_log.log_cnt = 0;
    cts_log.log_cnt += sprintf(buf+cts_log.log_cnt, "\nStart cts short test...\n");
    
    ret = cts_start_tiny_short_test(cts_dev, buf,tiny_short_config);
    err_num = tiny_short_config->short_tx_num + tiny_short_config->short_rx_num;
    
    if(ret){
        cts_info("!!! Cts short test error !!!");
        cts_log.log_cnt += sprintf(buf+cts_log.log_cnt, "!!! Cts short test error !!!\n");
        ret = -1;
    }else{
        if(tiny_short_config->tiny_short_result){
            cts_info("Cts short test PASS !");
            cts_log.log_cnt += sprintf(buf+cts_log.log_cnt, "!!! Cts short test PASS !!!\n");
            ret = 0;
        }else{
            cts_info("!!! Cts short test Fail  !!!");
            cts_log.log_cnt += sprintf(buf+cts_log.log_cnt, "!!! Cts short test Fail !!!\n");
            ret = err_num;
        }
    }
    cts_test_save_log(cts_dev,CTS_TEST_LOG_PATH,buf);
    kfree(buf);
    
    cts_dev->rtdata.testing = false;

    cts_plat_reset_device(cts_dev->pdata);

    cts_enter_normal_mode(cts_dev);

    cts_start_device(cts_dev);
    
    kfree(tiny_short_config);

    return ret;
    
}

/* Return 0 success 
          negative value while error occurs
          positive value means how many nodes fail */

int cts_rawdata_test(struct cts_device *cts_dev, u16* th_min, u16* th_max)
{
    int ret;
    char *buf = NULL;
    //u16 count = 0;
    
    cts_info("cts rawdata test");

   // cts_info("request rawdata test log memery size: %d", PAGE_SIZE);
    buf = (char*)kzalloc(PAGE_SIZE, GFP_KERNEL);
    if(buf == NULL){
        cts_err("allocate memery for rawdata test log fail");
        return -ENOMEM;
    }
    
    cts_log.log_cnt = 0;
    cts_log.log_cnt += sprintf(buf, "\nStart rawdata test...\n"); 

    ret = cts_rawdata_process(cts_dev, buf+cts_log.log_cnt, th_min, th_max, 0);
    if(ret < 0){
        cts_info("!!! Cts rawdata test error !!!");
        sprintf(buf+cts_log.log_cnt,"!!! Cts rawdata test error !!!\n");
    }else if(ret > 0){
        cts_info("!!! Cts rawdata test FAIL  !!!");
        sprintf(buf+cts_log.log_cnt,"!!! Cts rawdata test FAIL !!!\n");
    }else{
        cts_info("!!! Cts rawdata test PASS  !!!");
        sprintf(buf+cts_log.log_cnt,"!!! Cts rawdata test PASS !!!\n");
    }
    
    cts_test_save_log(cts_dev,CTS_TEST_LOG_PATH,buf);
    kfree(buf);
    return ret;
}

/* Return 0 success 
          negative value while error occurs
          positive value means how many nodes fail */

int cts_open_test(struct cts_device *cts_dev, u16 *th_min, u16 *th_max)
{
    int ret;
    char *buf = NULL;
    //u16 count = 0;
    
    cts_info("cts open test");
    
  //  cts_info("request log memery size: %d", PAGE_SIZE);
    buf = (char*)kzalloc(PAGE_SIZE, GFP_KERNEL);
    if(buf == NULL){
        cts_err("allocate memery for open test log fail");
        return -ENOMEM;
    }
    cts_log.log_cnt = 0;
    cts_log.log_cnt += sprintf(buf, "\nStart open test...\n"); 

	ret = cts_open_prepare(cts_dev);
	if(ret){
		cts_err("!!! Cts open test prepare error !!!");
		}
	
    ret = cts_rawdata_process(cts_dev, buf+cts_log.log_cnt, th_min, th_max, 1);
    if(ret < 0){
        cts_info("!!! Cts open test error !!!");
        sprintf(buf+cts_log.log_cnt,"!!! Cts open test error !!!\n");
    }else if(ret > 0){
        cts_info("!!! Cts open test FAIL  !!!");
        sprintf(buf+cts_log.log_cnt,"!!! Cts open test FAIL !!!\n");
    }else{
        cts_info("!!! Cts open test PASS  !!!");
        sprintf(buf+cts_log.log_cnt,"!!! Cts open test PASS !!!\n");
    }
	cts_plat_reset_device(cts_dev->pdata);
    cts_test_save_log(cts_dev, CTS_TEST_LOG_PATH, buf);
    kfree(buf);
    return ret;
}

int cts_test(struct cts_device *cts_dev, const char *filepath)
{
#define CTS_TEST_PASS   \
    "*********************************************\
    \n****************!!!PASSED!!!****************\
    \n*********************************************"
 #define CTS_TEST_FAIL   \
    "*********************************************\
    \n****************!!!FAILED!!!****************\
    \n*********************************************"
   
    struct file *file=NULL;
 //   char* test_config;
 //   u16 th;
 //   u16 size;
	u16 test_node;
//    int i ;
    int  ret;
    char *pass = CTS_TEST_PASS;
    char *fail = CTS_TEST_FAIL;
    char *buf = NULL;

    cts_info("Cts test");
    
    cts_stop_device(cts_dev);
    cts_dev->rtdata.testing = true;

	test_node = cts_dev->fwdata.rows * cts_dev->fwdata.cols;
#if 0
    if(filepath == NULL){
        test_config = cts_test_config_data;
        cts_info("Use cts test configure bultin driver");
    }else{
        cts_info("Open cts test cfg file '%s'", filepath);
        file = filp_open(filepath, O_RDONLY, 0);
        if (IS_ERR(file)) {
            cts_err("Open file '%s' failed %ld", filepath, PTR_ERR(file));
            return -1;
        }
      
        test_config = (char*)kzalloc(1024, GFP_KERNEL);
        if (test_config == NULL) {
            cts_err("Allocate test_config memery failed");
            goto cts_cfg_file_err;
            return -ENOMEM;
        }
        
        size = file_inode(file)->i_size;
        if(size <=0 && size > 1024){
            cts_err("Cts test config file invalid");
            goto cts_cfg_file_err;
        }
	

        if(cts_file_read(file, 0, test_config, size) < 0){
            cts_err("Read test config file error");
            goto cts_cfg_file_err;
        }
    }

    cts_info("cts_test_config:\n%s",test_config);
    
    for(i=0; i<ARRAY_SIZE(cts_test_items); i++){
        cts_info("cts test item[----step %d----] : %s", i, cts_test_items[i].item_name);
        ret = cts_parse_test_para(test_config, cts_test_items[i].item_name, &th);
        if(ret){
            cts_test_items[i].need_test = true;
        }else{
            cts_test_items[i].need_test = false;
        }

        if(cts_test_items[i].need_test){
            if(cts_test_items[i].para.para1){
                ret = cts_parse_test_para(test_config, cts_test_items[i].para.para1, &th);
                if(ret){
                   cts_test_items[i].para.threshold1 = th;
                   cts_info("Get test item para1: %s threshold1 = %d [0x%x]",cts_test_items[i].item_name, th,th);
                }else{
                    cts_err("Get test item para1: %s error!!! ",cts_test_items[i].item_name);
                    continue;
                }
            }
            if(cts_test_items[i].para.para2){
                ret = cts_parse_test_para(test_config, cts_test_items[i].para.para2, &th);
                if(ret){
                   cts_test_items[i].para.threshold2 = th;
                   cts_info("Get test item para2: %s threshold2 = %d [0x%x]",cts_test_items[i].item_name, th,th);
                }else{
                    cts_err("Get test item para2: %s error!!! ",cts_test_items[i].item_name);
                    continue;
                }
            }

            cts_test_items[i].result = cts_test_items[i].cts_test_fun(cts_dev, 
                cts_test_items[i].para.threshold1, cts_test_items[i].para.threshold2);
        }
        cts_info(" ");
    }
#endif

	if(cts_test_items[0].need_test){   // fw version test
        cts_test_items[0].result = cts_fw_version_test(cts_dev, TARGET_FW_VERSION);
    }

	if(cts_test_items[1].need_test){   // short test
        cts_test_items[1].result = cts_short_test(cts_dev, SHORT_TEST_TH);
    }

	if(cts_test_items[2].need_test){   // open test
		if( test_node > sizeof(open_rawdata_min) / sizeof(open_rawdata_min[0])
			|| test_node > sizeof(open_rawdata_max) / sizeof(open_rawdata_max[0]))
		{
			cts_err("rawdata threshod config error,rows = %d,cols = %d,arraysize_min = %d, arraysize_max = %d", cts_dev->fwdata.rows, 
					cts_dev->fwdata.cols, sizeof(open_rawdata_min) / sizeof(open_rawdata_min[0]), sizeof(open_rawdata_max) / sizeof(open_rawdata_max[0]));
			cts_test_items[3].result = 1;
		} else {
			cts_test_items[3].result = cts_open_test(cts_dev, open_rawdata_min, open_rawdata_max);
		}
    }

	if(cts_test_items[3].need_test){   // rawdata test
		if(test_node > sizeof(rawdata_test_min) / sizeof(rawdata_test_min[0])
			|| test_node > sizeof(rawdata_test_max) / sizeof(rawdata_test_max[0]))
		{
			cts_err("rawdata threshod config error,rows = %d,cols = %d,arraysize_min = %d, arraysize_max = %d", cts_dev->fwdata.rows, 
					cts_dev->fwdata.cols, sizeof(rawdata_test_min) / sizeof(rawdata_test_min[0]), sizeof(rawdata_test_max) / sizeof(rawdata_test_max[0]));
			cts_test_items[3].result = 1;
		} else {
			cts_test_items[3].result = cts_rawdata_test(cts_dev, rawdata_test_min, rawdata_test_max);
		}
    }

	ret = cts_test_items[0].result || cts_test_items[1].result 
		|| cts_test_items[2].result || cts_test_items[3].result;
    if(ret){
		cts_info(CTS_TEST_FAIL);
        buf = fail;
        
    }else{
        cts_info(CTS_TEST_PASS);
        buf = pass;
    }
    
    cts_test_save_log(cts_dev, CTS_TEST_LOG_PATH, buf);

    if(filepath != NULL){
      //  kfree(test_config);
        ret = filp_close(file, NULL);
        if (ret) {
            cts_warn("Close file '%s' failed %d", filepath, ret);
        }
    }

    cts_dev->rtdata.testing = false;

    cts_plat_reset_device(cts_dev->pdata);

    cts_enter_normal_mode(cts_dev);

    cts_start_device(cts_dev);

    return ret;

#if 0
cts_cfg_file_err:
        
    kfree(test_config);
//allocate_test_cfg_err:
    ret = filp_close(file, NULL);
    if (ret) {
        cts_warn("Close file '%s' failed %d", filepath, ret);
    }
    return -ENOMEM;
#endif
 #undef CTS_TEST_PASS
 #undef CTS_TEST_FAIL
}

#ifdef CTS_TOUCH_FACTORY_TEST
static ssize_t cts_test_read(struct file *file,
        char __user *buffer, size_t count, loff_t *ppos)
{
//	struct chipone_ts_data *cts_data;
	int ret = 0;
	if (*ppos) {
		return 0;
	}
	cts_info("cts_test_read");
#ifdef SUPPORT_TEST_CFG_FILE
    ret = cts_test(&g_cts_data->cts_dev, CTS_TEST_CFG_FILE_PATH);
#else
    ret = cts_test(&g_cts_data->cts_dev, NULL);
#endif
	if(ret){
		ret = copy_to_user(buffer, "FAIL\n", 5);
	}else{
		ret = copy_to_user(buffer, "Pass\n", 5);
	}
	*ppos += 5;
	return 5;
}

static ssize_t cts_test_write(struct file *file,
				const char __user * buffer, size_t count, loff_t * ppos)
{
	int ret  = 0;
	cts_info("cts_test_write");
    return ret;
}


static struct file_operations cts_factory_test_fops = {
    .owner = THIS_MODULE,
    .read  = cts_test_read,
    .write = cts_test_write,
};

int cts_factory_init(struct chipone_ts_data *cts_data)
{
    int ret = 0;

    cts_info("cts_factory Init");

    cts_data->proc_test_entry = proc_create_data(CFG_CTS_FACTORY_TEST_FILENAME,
            0666, NULL, &cts_factory_test_fops, cts_data);
    if (IS_ERR(cts_data->proc_test_entry)) {
        cts_err("Create proc test entry failed %ld",
            PTR_ERR(cts_data->proc_test_entry));
        cts_data->proc_test_entry = NULL;
    }
    return ret;
}

void cts_factory_deinit(struct chipone_ts_data *data)
{
    cts_info("cts_factory Deinit");

    if (data->proc_test_entry) {
        remove_proc_entry(CFG_CTS_FACTORY_TEST_FILENAME, NULL);
    }
}

#endif 

