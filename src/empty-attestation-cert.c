struct attestation_cert  __attribute__ ((section(".attestation.cert")))
attestation_cert = {
  .hdr = {
    .der_len = (uint32_t) -1,
    .der = NULL,
    .key = NULL
  }
};
