Archive pour l'exercice Titan #24.

Les fichiers modifiés se trouvent dans le répertoire Demos/Device/ClassDriver/Mouse

Pour télécharger le code dans la puce ATMega16u2 de l'Arduino UNO classique :

* Compiler le code en tapant la commande `make` dans le répertoire de l'exemple Demos/Device/ClassDriver/Mouse ;
* Court-circuiter deux broches du connecteur ISP de la puce ATMega16u2 (le connecteur mâle 3x2 broches le plus proche du connecteur USB),
  il s'agit des broches Reset et GND (les deux les plus proches du connecteur USB) ;
* L'Arduino UNO doit maintenant être listé en mode DFU par la commande `lsusb`, la programmation se fait par les commandes suivantes :

dfu-programmer atmega16u2 erase
dfu-programmer atmega16u2 flash Mouse.hex
dfu-programmer atmega16u2 reset
