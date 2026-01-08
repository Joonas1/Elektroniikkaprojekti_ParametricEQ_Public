ESP32 käynnistyy ensimmäisen kerran Access point tilassa, SSID: ESP32_Config, Password: 12345678

Yhdistämällä 192.168.4.1 voi asettaa Wifi SSID, salasana, firebase URL, firebase secret


Sama sivu on nyt saatavilla yhdistetyn verkon sisältä ESP32 ip-osoitteessa

Sinun pitää tehdä firebase projekti johon nettisivu ja esp32 kirjoittaa ja lukee tietoa

Nettisivu löytyy projektissa kansiosta "web" jonka voi hostata haluamallaan tavalla, kunhan se saa yhteyden firebaseen

Jotta ESP32 voi ohjata etänä pitää nettisivulla asetuksista laittaa firebase url ja secret oikein

Ensimmäinen käynnistys: ESP32 käynnistyy oletuksena Access Point -tilassa (tukiasema).

    SSID: ESP32_Config

    Salasana: 12345678

Asetukset: Yhdistä laitteeseen ja mene selaimella osoitteeseen 192.168.4.1. Täällä voit asettaa kotiverkkosi Wi-Fi-tiedot sekä Firebase URL:n ja Secret-avaimen.

<img width="502" height="564" alt="image" src="https://github.com/user-attachments/assets/eb4f7c71-cfbe-4935-b1fc-7127777d7030" />

Paikallinen käyttö: Kun ESP32 on yhdistetty verkkoon, konfigurointisivu on edelleen saatavilla ESP32:n omassa lähiverkon IP-osoitteessa.

Firebase-yhteys: Luo oma Firebase-projekti (Realtime Database). Jotta etäohjaus toimii, syötä sama Firebase URL ja Secret sekä ESP32-laitteelle että etäohjaussivustolle.

Etäohjaus: Projektin web-kansiosta löytyvän sivuston voi hostata itse. Muista asettaa Firebase-tiedot sivuston asetuksiin."
