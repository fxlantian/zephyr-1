#ifndef EMMC_CALLBACKCTL_H
#define EMMC_CALLBACKCTL_H

u32_t emmc_last_com_status(void);
u32_t *emmc_last_com_response(void);
u8_t *emmc_last_com_data(void);
u32_t emmc_last_cmd_status(void);
//emmc_postproc_callback emmc_get_post_callback(u32);
//emmc_preproc_callback emmc_get_pre_callback(u32);
void emmc_remove_command(void);
void emmc_set_data_trans_params(u32_t slot, u8_t * data_buffer,
				    u32_t num_of_blocks,
				    emmc_term_function
				    the_term_function,
				    emmc_copy_function the_copy_func,
				    u32_t epoch_count,
				    u32_t flag,u32_t custom_blocksize);

u32_t emmc_bus_corruption_present(void);

s32_t emmc_is_it_data_command(u32_t slot);

u32_t emmc_get_slave_intmask_task_status(u32_t slot);

void emmc_handle_standard_idsts(void * prv_data, u32_t int_status);

void emmc_handle_standard_rinsts(void *prv_data, u32_t int_status);

u32_t emmc_check_r1_resp(u32_t the_response);

u32_t emmc_check_r5_resp(u32_t the_resp);

s32_t emmc_get_qptr( u32_t * Status, u32_t * Buffer1, 
        u32_t * Buffer1_Virt, u32_t * Buffer2, u32_t * Buffer2_Virt);

u32_t emmc_write_out_data(current_task_status * the_task_status,
          u32_t the_interrupt_status);

u32_t emmc_read_in_data(current_task_status * the_task_status,
        u32_t the_interrupt_status);

u32_t emmc_send_serial_command(struct device *dev, u32_t slot, u32_t cmd_index, u32_t arg,
           u32_t * response_buffer, u8_t * data_buffer,
           u32_t flags,
           emmc_preproc_callback custom_preproc,
           emmc_postproc_callback custom_postproc);





typedef void (*irq_config_func_t)(struct device *dev);


struct emmc_callback_ppu_config {
    u32_t emmc_callback_base_addr;
    irq_config_func_t irq_config;
};

#endif				/* End of file */





