#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Provision slot 0 and export pubkey. Populates optional diagnostic rc values:
 *   get_rc    — last atcab_get_pubkey / read_pubkey attempt
 *   gen_rc    — last atcab_genkey attempt
 *   config_rc — config-zone write attempt (0 if skipped or unchanged)
 */
int vigia_atca_provision_slot0_verbose(
    uint8_t pubkey[64],
    bool *generated_new,
    int *get_rc,
    int *gen_rc,
    int *config_rc);

#ifdef __cplusplus
}
#endif
