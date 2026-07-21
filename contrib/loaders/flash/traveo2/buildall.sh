#!/bin/sh

for tgt in \
    TV2CE4MA0_S26HL512T \
    TV2CE4MA0_S27KL064;
do
  tgt=`basename ${tgt} .mk`
  echo "----- ${tgt} -----"
  make TARGET=${tgt} clean
  make TARGET=${tgt}
done
