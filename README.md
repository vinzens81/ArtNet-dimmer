# Art-Net Dimmer Node (ESP8266)

Ein-Kanal-Dimmer für Art-Net: der Node lauscht auf einem konfigurierbaren
Universe/Kanal und gibt den DMX-Wert per PWM auf einen LR7843-MOSFET aus.
Konfiguration, Status und ein Test-Slider laufen über eine kleine Weboberfläche,
Statusanzeige über ein zweifarbiges OLED.

## Hardware

| Teil | Pin am D1 mini | GPIO | Hinweis |
|---|---|---|---|
| LR7843 MOSFET-Modul, Signaleingang | `D1` | GPIO5 | PWM-Ausgang, active high |
| OLED SDA | `D6` | GPIO12 | I2C, Adresse `0x3C` |
| OLED SCL | `D5` | GPIO14 | |

Board ist ein **WeMos D1 mini** (ESP8266). Display: **0,96" 128×64 SSD1306/SSD1315**,
zweifarbig — die obersten 16 Pixelzeilen sind fest gelb, die restlichen 48 fest blau.
Das ist eine Eigenschaft des Panels, keine Software-Einstellung; das Layout hält
diese Grenze ein, damit keine Textzeile über beide Farben läuft.

> **Achtung, ungewohnte Pin-Belegung:** auf dem D1 mini sind `D1`/`D2` normalerweise
> die I2C-Pins. Hier belegt der MOSFET `D1`, deshalb ist I2C bewusst auf `D6`/`D5`
> verlegt (`Wire.begin(OLED_SDA, OLED_SCL)`).

### Verdrahtung MOSFET

Das LR7843-Modul schaltet **low side**, also die Masseleitung der Last. Der
Signaleingang (je nach Modul-Variante `SIG` oder `PWM` beschriftet) geht an `D1`,
und **ESP und Modul brauchen eine gemeinsame Masse** — sonst hat das Gate keinen
definierten Bezug und der MOSFET schaltet unzuverlässig. Die Last selbst wird
ausschließlich über die Schraubklemmen des Moduls geführt, nie über den ESP.

Bei induktiven Lasten (Motor, Magnetventil, Spule) gehört eine Freilaufdiode
parallel zur Last. Bei LEDs ist das nicht nötig.

## Dimmen

Der DMX-Wert (0–255) wird nicht linear auf den Duty-Cycle gelegt, sondern über
eine **Gamma-Kennlinie** (γ = 2.2, `buildGammaLut()`). Ohne das wirkt der untere
Bereich sprunghaft und der obere fast wirkungslos, weil die Helligkeitswahrnehmung
logarithmisch ist.

Grenzen der Hardware: der ESP8266 erzeugt PWM in Software, bei 1 kHz sind rund
1023 Stufen die Auflösungsgrenze (`PWM_RANGE`). Gamma 2.2 drückt die untersten
DMX-Werte damit rechnerisch auf 0 — die LUT hält deshalb eine Untergrenze von
Duty 1. Einen echten sichtbaren Startpunkt setzt man mit **Min-Helligkeit**.

* **Min-Helligkeit %** — Leistung bei DMX 1. Für Treiber, die erst spät zünden.
  DMX 0 ist immer hart aus, unabhängig davon.
* **Max-Helligkeit %** — Leistung bei DMX 255.
* **Fade ms** — weiches Nachziehen zwischen zwei Werten, 0 = aus.

Falls sichtbares Flackern auftritt (WLAN-Interrupts können den Software-PWM
stören): `PWM_FREQUENCY_HZ` auf ~500 senken oder `PWM_RANGE` auf 511.

## Erster Start

1. Flashen, Gerät startet im AP-Modus und zeigt das auf dem Display.
2. Mit dem WLAN **`ArtNet-Setup`** verbinden, das Captive Portal öffnet sich
   (sonst die im Display angezeigte IP im Browser aufrufen).
3. Eigenes WLAN auswählen, Passwort eingeben. Der Node bootet und zeigt danach
   seine IP an.
4. Web-Oberfläche öffnen und Universe/Kanal einstellen.

### WLAN zurücksetzen

Der Reset-Knopf liegt beim D1 mini direkt am EN-Pin, während er gedrückt ist läuft
kein Code — eine Haltedauer ist deshalb nicht messbar. Stattdessen:
**zweimal kurz hintereinander (< 4 s) resetten** löscht die WLAN-Zugangsdaten.
Das Flag liegt im RTC-Speicher, übersteht also einen Reset, aber keinen
Stromausfall.

## Weboberfläche

Erreichbar über die IP oder per mDNS als `<name>.local` (Standard:
`artnet-dimmer.local`). Der Hostname wird aus dem Gerätenamen abgeleitet und
gilt erst nach einem Neustart.

Die Seite ist statisch und holt alle Werte per JSON, zweimal pro Sekunde.

| Route | Methode | Zweck |
|---|---|---|
| `/` | GET | Oberfläche |
| `/status` | GET | JSON: IP, Name, Universe, Kanal, DMX-Wert, Ausgang %, Flags |
| `/set?v=0..255` | GET | Test-Slider, schaltet in den Manuell-Modus |
| `/set?auto=1` | GET | Manuell-Modus verlassen, wieder DMX folgen |
| `/save` | POST | Konfiguration speichern (form-encoded) |

Der **Test-Slider** steuert den Ausgang ohne Controller. Solange der
Manuell-Modus aktiv ist, werden eintreffende DMX-Werte mitgelesen und angezeigt,
aber nicht ausgegeben — „Wieder DMX folgen" gibt die Kontrolle zurück.

## Display

* **Gelber Streifen:** IP-Adresse, darunter Universe/Kanal und rechts ein
  Status-Kürzel — `DMX` (Daten kommen an), `MAN` (Manuell-Modus) oder `--`
  (kein DMX).
* **Blauer Bereich:** Balken mit dem rohen 8-Bit-DMX-Wert mittig (per XOR
  gezeichnet, deshalb im gefüllten wie im leeren Teil lesbar), darunter die
  tatsächliche Ausgangsleistung in Prozent nach Gamma und Trim.

Der Refresh ist auf 10 Hz gedrosselt und läuft aus `loop()`, nicht aus dem
DMX-Pfad: ein voller SSD1306-Refresh dauert ~25 ms, Art-Net kommt alle ~23 ms —
im Empfangspfad würde das Display den Node ausbremsen.

Fehlt das Display, wird das beim Start per I2C erkannt, auf Serial gemeldet, und
die Firmware läuft ohne weiter.

## Art-Net

* Port **6454**, `ArtDmx` (OpCode `0x5000`) und `ArtPoll` (`0x2000`) werden
  behandelt, auf `ArtPoll` antwortet der Node mit einem `ArtPollReply`.
* Die Port-Address wird über die **vollen 15 Bit** ausgewertet
  (Net/Subnet/Universe, 0–32767) — nicht nur die unteren 4 Bit.
* Die **Sequence-Nummer** wird geprüft, damit von UDP umgeordnete Pakete keinen
  neueren Wert überschreiben. `0` heißt laut Spec „Sender nutzt keine Sequenz";
  nach einer Pause wird die Folge nicht mehr bewertet, damit ein Sender-Neustart
  den Node nicht blockiert.
* **Signalausfall:** kommen 4 Sekunden keine Pakete mehr, geht der Ausgang auf 0
  und das Display zeigt `--`. Ohne das würde der MOSFET nach einem Absturz der
  Konsole auf dem letzten Wert stehen bleiben.

## Konfiguration

Wird im EEPROM gehalten (`struct Config`), abgesichert über ein Magic-Word.
Passt das Magic nicht, werden Standardwerte geschrieben — **bei einer Änderung
der Struct-Felder muss `CONFIG_MAGIC` hochgezählt werden**, sonst wird alter
Inhalt falsch interpretiert. Aktuell: `0xC0FFEE03`.

| Feld | Bereich | Standard |
|---|---|---|
| Name | max. 17 Zeichen | `ArtNet Dimmer` |
| Universe | 0–32767 | 0 |
| Kanal | 1–512 | 1 |
| Min-Helligkeit | 0–100 % | 0 |
| Max-Helligkeit | 0–100 % | 100 |
| Fade | 0–5000 ms | 0 |

Die WLAN-Zugangsdaten liegen separat, verwaltet von WiFiManager.

## Build & Flash

```sh
pio run              # kompilieren
pio run -t upload    # über USB flashen
pio device monitor   # Serial, 115200 Baud
```

**OTA:** nach dem ersten USB-Flash lässt sich die Firmware über WLAN aktualisieren.
Beim Update-Start fährt der Ausgang auf 0 und das Display zeigt „OTA-Update", weil
während des Schreibens kein PWM-Timer läuft. Für Uploads aus PlatformIO in
`platformio.ini` ergänzen:

```ini
upload_protocol = espota
upload_port = artnet-dimmer.local
```

OTA ist **ohne Passwort** konfiguriert — für ein Gerät im vertrauenswürdigen
Netz. Absichern mit `ArduinoOTA.setPassword("...")` in `setup()`.

## Wichtige Konstanten

Alle am Anfang von `src/main.cpp`:

| Konstante | Bedeutung |
|---|---|
| `MOSFET_PIN` | PWM-Ausgang zum MOSFET |
| `MOSFET_ACTIVE_LOW` | auf `true`, falls das Modul invertiert ansteuert |
| `PWM_RANGE`, `PWM_FREQUENCY_HZ` | Auflösung und Frequenz des Software-PWM |
| `GAMMA` | Dimmkurve |
| `DMX_TIMEOUT_MS` | Ausfallzeit bis der Ausgang abschaltet |
| `DISPLAY_MIN_INTERVAL_MS` | Drosselung des Display-Refresh |
| `DEBUG_DMX` | Logging pro Paket; kostet ~5 ms und bremst den Empfang |
| `OLED_YELLOW_H` | Höhe des gelben Bereichs, falls das Panel anders aufgeteilt ist |
