/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */


// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/sim_interface.h"
#include "acados_sim_solver_Jackal.h"

#define NX     JACKAL_NX
#define NZ     JACKAL_NZ
#define NU     JACKAL_NU
#define NP     JACKAL_NP


int main()
{
    int status = 0;
    Jackal_sim_solver_capsule *capsule = Jackal_acados_sim_solver_create_capsule();
    status = Jackal_acados_sim_create(capsule);

    if (status)
    {
        printf("acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    sim_config *acados_sim_config = Jackal_acados_get_sim_config(capsule);
    sim_in *acados_sim_in = Jackal_acados_get_sim_in(capsule);
    sim_out *acados_sim_out = Jackal_acados_get_sim_out(capsule);
    void *acados_sim_dims = Jackal_acados_get_sim_dims(capsule);

    // initial condition
    double x_current[NX];
    x_current[0] = 0.0;
    x_current[1] = 0.0;
    x_current[2] = 0.0;

  
    x_current[0] = 0;
    x_current[1] = 0;
    x_current[2] = 0.7853981633974483;
    
  


    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;
    // set parameters
    double p[NP];
    p[0] = -100;
    p[1] = -100;
    p[2] = -100;
    p[3] = -100;
    p[4] = -100;
    p[5] = -100;
    p[6] = -100;
    p[7] = -100;
    p[8] = -100;
    p[9] = -100;
    p[10] = -100;
    p[11] = -100;
    p[12] = -100;
    p[13] = -100;
    p[14] = -100;
    p[15] = -100;
    p[16] = -100;
    p[17] = -100;
    p[18] = -100;
    p[19] = -100;
    p[20] = -100;
    p[21] = -100;
    p[22] = -100;
    p[23] = -100;
    p[24] = -100;
    p[25] = -100;
    p[26] = -100;
    p[27] = -100;
    p[28] = -100;
    p[29] = -100;
    p[30] = -100;
    p[31] = -100;
    p[32] = -100;
    p[33] = -100;
    p[34] = -100;
    p[35] = -100;
    p[36] = -100;
    p[37] = -100;
    p[38] = -100;
    p[39] = -100;
    p[40] = -100;
    p[41] = -100;
    p[42] = -100;
    p[43] = -100;
    p[44] = -100;
    p[45] = -100;
    p[46] = -100;
    p[47] = -100;
    p[48] = -100;
    p[49] = -100;
    p[50] = -100;
    p[51] = -100;
    p[52] = -100;
    p[53] = -100;
    p[54] = -100;
    p[55] = -100;
    p[56] = -100;
    p[57] = -100;
    p[58] = -100;
    p[59] = -100;
    p[60] = -100;
    p[61] = -100;
    p[62] = -100;
    p[63] = -100;
    p[64] = -100;
    p[65] = -100;
    p[66] = -100;
    p[67] = -100;
    p[68] = -100;
    p[69] = -100;
    p[70] = -100;
    p[71] = -100;
    p[72] = -100;
    p[73] = -100;
    p[74] = -100;
    p[75] = -100;
    p[76] = -100;
    p[77] = -100;
    p[78] = -100;
    p[79] = -100;
    p[80] = -100;
    p[81] = -100;
    p[82] = -100;
    p[83] = -100;
    p[84] = -100;
    p[85] = -100;
    p[86] = -100;
    p[87] = -100;
    p[88] = -100;
    p[89] = -100;
    p[90] = -100;
    p[91] = -100;
    p[92] = -100;
    p[93] = -100;
    p[94] = -100;
    p[95] = -100;
    p[96] = -100;
    p[97] = -100;
    p[98] = -100;
    p[99] = -100;
    p[100] = -100;
    p[101] = -100;
    p[102] = -100;
    p[103] = -100;
    p[104] = -100;
    p[105] = -100;
    p[106] = -100;
    p[107] = -100;
    p[108] = -100;
    p[109] = -100;
    p[110] = -100;
    p[111] = -100;
    p[112] = -100;
    p[113] = -100;
    p[114] = -100;
    p[115] = -100;
    p[116] = -100;
    p[117] = -100;
    p[118] = -100;
    p[119] = -100;
    p[120] = -100;
    p[121] = -100;
    p[122] = -100;
    p[123] = -100;
    p[124] = -100;
    p[125] = -100;
    p[126] = -100;
    p[127] = -100;
    p[128] = -100;
    p[129] = -100;
    p[130] = -100;
    p[131] = -100;
    p[132] = -100;
    p[133] = -100;
    p[134] = -100;
    p[135] = -100;
    p[136] = -100;
    p[137] = -100;
    p[138] = -100;
    p[139] = -100;
    p[140] = -100;
    p[141] = -100;
    p[142] = -100;
    p[143] = -100;
    p[144] = -100;
    p[145] = -100;
    p[146] = -100;
    p[147] = -100;
    p[148] = -100;
    p[149] = -100;
    p[150] = -100;
    p[151] = -100;
    p[152] = -100;
    p[153] = -100;
    p[154] = -100;
    p[155] = -100;
    p[156] = -100;
    p[157] = -100;
    p[158] = -100;
    p[159] = -100;
    p[160] = -100;
    p[161] = -100;
    p[162] = -100;
    p[163] = -100;
    p[164] = -100;
    p[165] = -100;
    p[166] = -100;
    p[167] = -100;
    p[168] = -100;
    p[169] = -100;
    p[170] = -100;
    p[171] = -100;
    p[172] = -100;
    p[173] = -100;
    p[174] = -100;
    p[175] = -100;
    p[176] = -100;
    p[177] = -100;
    p[178] = -100;
    p[179] = -100;
    p[180] = -100;
    p[181] = -100;
    p[182] = -100;
    p[183] = -100;
    p[184] = -100;
    p[185] = -100;
    p[186] = -100;
    p[187] = -100;
    p[188] = -100;
    p[189] = -100;
    p[190] = -100;
    p[191] = -100;
    p[192] = -100;
    p[193] = -100;
    p[194] = -100;
    p[195] = -100;
    p[196] = -100;
    p[197] = -100;
    p[198] = -100;
    p[199] = -100;
    p[200] = -100;
    p[201] = -100;
    p[202] = -100;
    p[203] = -100;
    p[204] = -100;
    p[205] = -100;
    p[206] = -100;
    p[207] = -100;
    p[208] = -100;
    p[209] = -100;
    p[210] = -100;
    p[211] = -100;
    p[212] = -100;
    p[213] = -100;
    p[214] = -100;
    p[215] = -100;
    p[216] = -100;
    p[217] = -100;
    p[218] = -100;
    p[219] = -100;
    p[220] = -100;
    p[221] = -100;
    p[222] = -100;
    p[223] = -100;
    p[224] = -100;
    p[225] = -100;
    p[226] = -100;
    p[227] = -100;
    p[228] = -100;
    p[229] = -100;
    p[230] = -100;
    p[231] = -100;
    p[232] = -100;
    p[233] = -100;
    p[234] = -100;
    p[235] = -100;
    p[236] = -100;
    p[237] = -100;
    p[238] = -100;
    p[239] = -100;
    p[240] = -100;
    p[241] = -100;
    p[242] = -100;
    p[243] = -100;
    p[244] = -100;
    p[245] = -100;
    p[246] = -100;
    p[247] = -100;
    p[248] = -100;
    p[249] = -100;
    p[250] = -100;
    p[251] = -100;
    p[252] = -100;
    p[253] = -100;
    p[254] = -100;
    p[255] = -100;
    p[256] = -100;
    p[257] = -100;
    p[258] = -100;
    p[259] = -100;
    p[260] = -100;
    p[261] = -100;
    p[262] = -100;
    p[263] = -100;
    p[264] = -100;
    p[265] = -100;
    p[266] = -100;
    p[267] = -100;
    p[268] = -100;
    p[269] = -100;
    p[270] = -100;
    p[271] = -100;
    p[272] = -100;
    p[273] = -100;
    p[274] = -100;
    p[275] = -100;
    p[276] = -100;
    p[277] = -100;
    p[278] = -100;
    p[279] = -100;
    p[280] = -100;
    p[281] = -100;
    p[282] = -100;
    p[283] = -100;
    p[284] = -100;
    p[285] = -100;
    p[286] = -100;
    p[287] = -100;
    p[288] = -100;
    p[289] = -100;
    p[290] = -100;
    p[291] = -100;
    p[292] = -100;
    p[293] = -100;
    p[294] = -100;
    p[295] = -100;
    p[296] = -100;
    p[297] = -100;
    p[298] = -100;
    p[299] = -100;
    p[300] = -100;
    p[301] = -100;
    p[302] = -100;
    p[303] = -100;
    p[304] = -100;
    p[305] = -100;
    p[306] = -100;
    p[307] = -100;
    p[308] = -100;
    p[309] = -100;
    p[310] = -100;
    p[311] = -100;
    p[312] = -100;
    p[313] = -100;
    p[314] = -100;
    p[315] = -100;
    p[316] = -100;
    p[317] = -100;
    p[318] = -100;
    p[319] = -100;
    p[320] = -100;
    p[321] = -100;
    p[322] = -100;
    p[323] = -100;
    p[324] = -100;
    p[325] = -100;
    p[326] = -100;
    p[327] = -100;
    p[328] = -100;
    p[329] = -100;
    p[330] = -100;
    p[331] = -100;
    p[332] = -100;
    p[333] = -100;
    p[334] = -100;
    p[335] = -100;
    p[336] = -100;
    p[337] = -100;
    p[338] = -100;
    p[339] = -100;
    p[340] = -100;
    p[341] = -100;
    p[342] = -100;
    p[343] = -100;
    p[344] = -100;
    p[345] = -100;
    p[346] = -100;
    p[347] = -100;
    p[348] = -100;
    p[349] = -100;
    p[350] = -100;
    p[351] = -100;
    p[352] = -100;
    p[353] = -100;
    p[354] = -100;
    p[355] = -100;
    p[356] = -100;
    p[357] = -100;
    p[358] = -100;
    p[359] = -100;
    p[360] = -100;
    p[361] = -100;
    p[362] = -100;
    p[363] = -100;
    p[364] = -100;
    p[365] = -100;
    p[366] = -100;
    p[367] = -100;
    p[368] = -100;
    p[369] = -100;
    p[370] = -100;
    p[371] = -100;
    p[372] = -100;
    p[373] = -100;
    p[374] = -100;
    p[375] = -100;
    p[376] = -100;
    p[377] = -100;
    p[378] = -100;
    p[379] = -100;
    p[380] = -100;
    p[381] = -100;
    p[382] = -100;
    p[383] = -100;
    p[384] = -100;
    p[385] = -100;
    p[386] = -100;
    p[387] = -100;
    p[388] = -100;
    p[389] = -100;
    p[390] = -100;
    p[391] = -100;
    p[392] = -100;
    p[393] = -100;
    p[394] = -100;
    p[395] = -100;
    p[396] = -100;
    p[397] = -100;
    p[398] = -100;
    p[399] = -100;
    p[400] = -100;
    p[401] = -100;
    p[402] = -100;
    p[403] = -100;
    p[404] = -100;
    p[405] = -100;
    p[406] = -100;
    p[407] = -100;
    p[408] = -100;
    p[409] = -100;
    p[410] = -100;
    p[411] = -100;
    p[412] = -100;
    p[413] = -100;
    p[414] = -100;
    p[415] = -100;
    p[416] = -100;
    p[417] = -100;
    p[418] = -100;
    p[419] = -100;
    p[420] = -100;
    p[421] = -100;
    p[422] = -100;
    p[423] = -100;
    p[424] = -100;
    p[425] = -100;
    p[426] = -100;
    p[427] = -100;
    p[428] = -100;
    p[429] = -100;
    p[430] = -100;
    p[431] = -100;
    p[432] = -100;
    p[433] = -100;
    p[434] = -100;
    p[435] = -100;
    p[436] = -100;
    p[437] = -100;
    p[438] = -100;
    p[439] = -100;
    p[440] = -100;
    p[441] = -100;
    p[442] = -100;
    p[443] = -100;
    p[444] = -100;
    p[445] = -100;
    p[446] = -100;
    p[447] = -100;
    p[448] = -100;
    p[449] = -100;
    p[450] = -100;
    p[451] = -100;
    p[452] = -100;
    p[453] = -100;
    p[454] = -100;
    p[455] = -100;
    p[456] = -100;
    p[457] = -100;
    p[458] = -100;
    p[459] = -100;
    p[460] = -100;
    p[461] = -100;
    p[462] = -100;
    p[463] = -100;
    p[464] = -100;
    p[465] = -100;
    p[466] = -100;
    p[467] = -100;
    p[468] = -100;
    p[469] = -100;
    p[470] = -100;
    p[471] = -100;
    p[472] = -100;
    p[473] = -100;
    p[474] = -100;
    p[475] = -100;
    p[476] = -100;
    p[477] = -100;
    p[478] = -100;
    p[479] = -100;
    p[480] = -100;
    p[481] = -100;
    p[482] = -100;
    p[483] = -100;
    p[484] = -100;
    p[485] = -100;
    p[486] = -100;
    p[487] = -100;
    p[488] = -100;
    p[489] = -100;
    p[490] = -100;
    p[491] = -100;
    p[492] = -100;
    p[493] = -100;
    p[494] = -100;
    p[495] = -100;
    p[496] = -100;
    p[497] = -100;
    p[498] = -100;
    p[499] = -100;
    p[500] = -100;
    p[501] = -100;
    p[502] = -100;
    p[503] = -100;
    p[504] = -100;
    p[505] = -100;
    p[506] = -100;
    p[507] = -100;
    p[508] = -100;
    p[509] = -100;
    p[510] = -100;
    p[511] = -100;
    p[512] = -100;
    p[513] = -100;
    p[514] = -100;
    p[515] = -100;
    p[516] = -100;
    p[517] = -100;
    p[518] = -100;
    p[519] = -100;

    Jackal_acados_sim_update_params(capsule, p, NP);
  

  


    int n_sim_steps = 3;
    // solve ocp in loop
    for (int ii = 0; ii < n_sim_steps; ii++)
    {
        // set inputs
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "x", x_current);
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "u", u0);

        // solve
        status = Jackal_acados_sim_solve(capsule);
        if (status != ACADOS_SUCCESS)
        {
            printf("acados_solve() failed with status %d.\n", status);
        }

        // get outputs
        sim_out_get(acados_sim_config, acados_sim_dims,
               acados_sim_out, "x", x_current);

    

        // print solution
        printf("\nx_current, %d\n", ii);
        for (int jj = 0; jj < NX; jj++)
        {
            printf("%e\n", x_current[jj]);
        }
    }

    printf("\nPerformed %d simulation steps with acados integrator successfully.\n\n", n_sim_steps);

    // free solver
    status = Jackal_acados_sim_free(capsule);
    if (status) {
        printf("Jackal_acados_sim_free() returned status %d. \n", status);
    }

    Jackal_acados_sim_solver_free_capsule(capsule);

    return status;
}
