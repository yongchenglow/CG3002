from Crypto.Cipher import AES
import base64
import sys
import os

class server_auth:
    def __init__(self):
        super(server_auth, self).__init__()

    def decryptText(self, cipherText, Key):
        decodedMSG = base64.b64decode(cipherText)
        iv = decodedMSG[:16]
        secret_key = Key; 
        cipher = AES.new(secret_key,AES.MODE_CBC,iv)
        decryptedText = cipher.decrypt(decodedMSG[16:]).strip()
        decryptedTextStr = decryptedText.decode('utf8')
        decryptedTextStr1 = decryptedTextStr[decryptedTextStr.find('#'):] 
        decryptedTextFinal = bytes(decryptedTextStr1[1:],'utf8').decode('utf8')
        action = decryptedTextFinal.split('|')[0]
        voltage = decryptedTextFinal.split('|')[1]
        current = decryptedTextFinal.split('|')[2]
        power = decryptedTextFinal.split('|')[3]
        cumpower = decryptedTextFinal.split('|')[4]
        return {'action': action, 'voltage': voltage, 'current': current, 'power': power, 'cumpower': cumpower}





