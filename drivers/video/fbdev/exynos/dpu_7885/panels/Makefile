obj-$(CONFIG_EXYNOS_DECON_LCD_S6E3FA0)	+= s6e3fa0_mipi_lcd.o s6e3fa0_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_S6E3FA3)	+= s6e3fa3_mipi_lcd.o s6e3fa3_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_S6E3FA7)	+= s6e3fa7_mipi_lcd.o s6e3fa7_lcd_ctrl.o

obj-y += dsim_panel.o ../decon_board.o ../decon_notify.o ../decon_abd.o
obj-$(CONFIG_DISPLAY_USE_INFO) += dpui.o
obj-$(CONFIG_EXYNOS_DECON_MDNIE) += mdnie_lite.o
obj-$(CONFIG_EXYNOS_DECON_MDNIE_CONTROL) += mdnie_control.o

obj-$(and $(CONFIG_EXYNOS_DECON_LCD_S6E3FA7_A7Y18),$(CONFIG_SUPPORT_POC_FLASH)) += poc.o
obj-$(and $(CONFIG_EXYNOS_DECON_LCD_EA8076_A30),$(CONFIG_SUPPORT_POC_FLASH)) += ea8076_a30_poc.o

obj-$(and $(CONFIG_EXYNOS_DECON_MDNIE),$(CONFIG_SEC_GPIO_DVS),$(subst undefined,y,$(origin CONFIG_SAMSUNG_PRODUCT_SHIP))) += dd_mdnie.o
obj-$(and $(CONFIG_SEC_GPIO_DVS),$(subst undefined,y,$(origin CONFIG_SAMSUNG_PRODUCT_SHIP))) += dd_lcd.o dd_backlight.o dd_dpu.o dd_param.o

obj-$(CONFIG_EXYNOS_DECON_LCD_S6E3FA7_JACKPOT2)	+= s6e3fa7_jackpot_lcd_ctrl.o dynamic_aid.o
obj-$(CONFIG_EXYNOS_DECON_LCD_S6E3FA7_JACKPOT)	+= s6e3fa7_jackpot_lcd_ctrl.o dynamic_aid.o
obj-$(CONFIG_EXYNOS_DECON_LCD_TD4100_J3TOPE)	+= td4100_j3tope_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_S6D7AT0B_J7TOPE)	+= s6d7at0b_j7tope_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_EA8061S_J7DUO)	+= ea8061s_j7duo_lcd_ctrl.o dynamic_aid.o
obj-$(CONFIG_EXYNOS_DECON_LCD_S6E8AA5_A6ELTE)	+= s6e8aa5_a6elte_lcd_ctrl.o dynamic_aid.o
obj-$(CONFIG_EXYNOS_DECON_LCD_S6E8AA5_FEEL2)	+= s6e8aa5_a6elte_lcd_ctrl.o dynamic_aid.o
obj-$(CONFIG_EXYNOS_DECON_LCD_SN65DSI86_GVIEW2)	+= sn65dsi86_gview2_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_S6E3FA7_A7Y18)	+= s6e3fa7_a7y18_lcd_ctrl.o dynamic_aid.o
obj-$(CONFIG_EXYNOS_DECON_LCD_TD4101_J2COREPELTE)	+= td4101_j2corepelte_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_HX8279D_GTA3XLLTE)	+= hx8279d_gta3xllte_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_HX83112A_M20)	+= hx83112a_m20_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_EA8076_M30)	+= ea8076_m30_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_EA8076_A30)	+= ea8076_a30_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_HX8279D_WISDOM)	+= hx8279d_wisdom_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_S6E8FC0_A20)	+= s6e8fc0_a20_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_S6E8FC0_A40)	+= s6e8fc0_a40_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_S6D7AT0B_A10)	+= s6d7at0b_a10_lcd_ctrl.o s6d7at0b_a10_boe_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_HX83102D_A10E)	+= hx83102d_a10e_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_HX83102D_A20E)	+= hx83102d_a20e_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_S6D7AA0_XCOVER4S)	+= s6d7aa0_xcover4s_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_S6E8FC0_A30C)	+= s6e8fc0_a30c_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_S6E8FC1_A30S)	+= s6e8fc1_a30s_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_HX83102D_A10E_JPN)	+= hx83102d_a10e_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_HX83102P_GTA4S)	+= hx83102p_gta4s_lcd_ctrl.o
