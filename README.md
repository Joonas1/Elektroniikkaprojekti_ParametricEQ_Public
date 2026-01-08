Ensimmäinen käynnistys: ESP32 käynnistyy oletuksena Access Point -tilassa (tukiasema).

    SSID: ESP32_Config

    Salasana: 12345678

Asetukset: Yhdistä laitteeseen ja mene selaimella osoitteeseen 192.168.4.1. Täällä voit asettaa kotiverkkosi Wi-Fi-tiedot sekä Firebase URL:n ja Secret-avaimen.

<img width="502" height="564" alt="image" src="https://github.com/user-attachments/assets/eb4f7c71-cfbe-4935-b1fc-7127777d7030" />

Paikallinen käyttö: Kun ESP32 on yhdistetty verkkoon, konfigurointisivu on edelleen saatavilla ESP32:n omassa lähiverkon IP-osoitteessa.

Firebase-yhteys: Luo oma Firebase-projekti (Realtime Database). Jotta etäohjaus toimii, syötä sama Firebase URL ja Secret sekä ESP32-laitteelle että etäohjaussivustolle.

Etäohjaus: Projektin web-kansiosta löytyvän sivuston voi hostata itse. Muista asettaa Firebase-tiedot sivuston asetuksiin."
