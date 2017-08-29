rm l*
rm f*
rm t*
rm spi_stim.txt
rm zephyr.s19
cp ../outdir/artix7_ppu/zephyr.s19 ./
./s19toslm.py zephyr.s19
cp spi_stim.txt ~/
