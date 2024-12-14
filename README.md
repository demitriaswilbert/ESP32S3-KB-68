# Scan firmware
## Proper bounce detection
- Implemented a proper bounce detection to minimize double pressing the key
- 100 ms key timeout
## Non comutative layering
- If 2 or more keys each have alternate keycodes from another layer,\
you can still simultaneously press the first keycode from the first layer and the second keycode from the second layer

- ex: lets say keys 1 and 2 have keycodes F1 and F2 on the second layer\
if you press in order:
    -     1, fn, 2
  the keyboard would send:
    -     1 and F2
  instead of
    -     F1 and F2

## 1000Hz scan rate & poll rate
- yes
## Auto typing
- Enable a mode where you press a key and it sends some text
    -     To activate / deactivate : FN + RCTRL + Left Arrow + Right Arrow

## working capslock & numlock led
- when the capslock / numlock is pressed, the keyboard receives an output event

## Multimedia features
- raise / lower volume, brightness, open calculator, email, etc

# USB Serial configuration
## autotyping memory configuration
- Enter some text, then press \033 (escape) to keep the text

## configuration via usb serial 
- Set the keyboard behavior (Send Esc after each statement)
    - to set output target `x = (1: usb, 2: wireless, 3: both)`:
        -     "KB target x"
    - To set delay between each key `x in ms`:
        -     "KB delay x"
    - To set text position `x in text index`
        -     "KB pos x"
    - Get peer (receiver) ESP-32 MAC address
        -     "KB getmac"
    - Set peer (receiver) ESP-32 MAC address
        -     "KB setmac"
    - Get the ESP-32 MAC address from this device
        -     "KB mac"
    - Load saved text from SPIFFS:
        -     "KB load"
    - Save text to SPIFFS:
        -     "KB save"
          
# ESP-32 advantage
- Wireless 2.4G
    - Using ESP-NOW protocol
- Low Power sleep mode
