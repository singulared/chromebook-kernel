#ifndef __SETUP_CHARGER_H
#define __SETUP_CHARGER_H __FILE__

extern void nuri_chg_irqs_set_base(int pmic_base, int fuel_gauge_base);
extern struct platform_device nuri_charger_manager;
extern struct charger_global_desc nuri_charger_g_desc;

#endif /* __SETUP_CHARGER_H */
