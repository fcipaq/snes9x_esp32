#pragma once

#define PWR_ON_UNKOWN 0
#define PWR_ON_BAT 1
#define PWR_ON_GRD 2

void pwr_init();
void pwr_shutdown();
void pwr_amp(int on);
void pwr_sd(int on);
bool pwr_sd_det();

int pwr_get_bat_vol();
int pwr_get_bat_level(int granularity);
int pwr_get_source();
int pwr_get_src_vol();

// fix me
void snd_mute();
void snd_unmute();

