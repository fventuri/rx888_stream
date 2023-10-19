# rx888_stream
Outputs samples to stdout. To build run `make`

Example command: `./rx888_stream -f SDDC_FX3.img --samplerate 32000000`

Allows control of reference clock and clock correction in ppm; for instance:
`./rx888_stream -f SDDC_FX3.img --samplerate 32000000 --correction -1.45`

## Credits

  - [rx888_test](https://github.com/cozycactus/rx888_test)
  - [original rx888_stream](https://github.com/rhgndf/rx888_stream)

## License

Parts of the code (functions 'start_adc' and 'rational_approximation') are licensed under the GNU GLP V3 (see [LICENSE](LICENSE))
