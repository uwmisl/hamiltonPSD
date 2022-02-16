from time import sleep
import serial
import os
from functools import reduce

class PSD8(object):
  """
    PSD8 (and possibly others) protocol wrapper class.
  """

  def __init__(self,port="COM6",baud=9600, speed = 7, debug = False):
    """ Interface for PSD8
      port(str): com port that the pump is plugged into
      baud (int): baud rate
      speed (int 1-40): default speed.
    """
    self.debug = debug
    self.default_speed = speed
    self.speed = None #unknown at this point
    if not self.debug:
      self.spt = serial.Serial(port=port,
                               baudrate=baud,
                               timeout=1,
                               inter_byte_timeout = 0.01,
                               exclusive=True)
    self.sequence = 1 #transmission sequence number
    self.error_codes = {}
    pth = os.path.dirname(os.path.abspath(__file__))
    with open(pth+"/"+"error_codes.csv",'rb') as codes:
      for line in codes:
        toks = line.split(b',')
        key = int(toks[0])
        error = toks[1].strip()
        self.error_codes[key] = error
    self.set_speed(self.default_speed)

  @staticmethod
  def _checksum(s):
    """ returns checksum as int """
    return reduce(lambda x,y:x^(ord(y)), str(s), 0)

  def _send(self,cmd,address=1):
    """TODO: handle retrys and sequence numbers correctly

    """
    retry_count = 10 #TODO: parameterize?
    repeat = 0

    print("enter _send")
    packet = (b"\x02"+chr(ord("0")+address).encode()+
              chr( (self.sequence+ord("0"))|(0b00001000*repeat) ).encode() +
              cmd+b"\x03")
    print(11)
    checksum = self._checksum(packet)
    packet += chr(checksum).encode()

    print(12)
    while retry_count>0:
      print(13)
      self.spt.reset_input_buffer()
      print(14)
      self.spt.write(packet)
      print(15)
      in_packet = b""

      #how many times should we keep trying for data if we don't see the end?
      read_count = 10
      while (len(in_packet)== 0 or in_packet[-2] != b"\x03") and read_count>0:
        print(in_packet)
        sleep(8.0*len(packet)/self.spt.baudrate+0.01)
        read_count -=1
        print(16)
        in_packet += self.spt.read(100)
        print(17)

      #I have no idea where the extra 0xff byte is coming from
      in_packet = in_packet.lstrip(b"\xff")
      try:
        #throws value error if there is a bad or no packet
        self._check_packet(in_packet)
        #sucessful transation! (may still have error reported)
        #update next sequence number
        self.sequence = (self.sequence+1,1)[self.sequence>=7]
        return in_packet
      except:
        retry_count -= 1
        #next packets will all be repeats.  set repeat and don't inc sequence
        repeat = 1
    #FAILED TO SEND!  TODO: choose correct exception
    raise Exception("Retry limit reached: failed to send command: "+packet)

  def _check_packet(self,packet):
    """
      packet (str): data packet from pump to computer

      returns: status dict {"ready": true for ready,
                            "errorStatus": error condition code}

      throws: ValueError if packet is bad.
    """
    print("enter check packet")
    #ensure it's addressed to me and complete
    if (not packet.startswith("\x020")) or packet[-2] != "\x03":
      raise ValueError("must be a complete response packet.  got: {}".format(str( map(ord,packet))))
    statusbyte = ord(packet[2])
    #check checksum
    if self._checksum(packet[:-1]) != ord(packet[-1]):
      raise ValueError("failed checksum")
    #check constant fields
    if statusbyte&0b11010000 != 0b01000000:
      raise ValueError("status byte invalid")

    return {"ready": bool(0b00100000 & statusbyte),
            "errorStatus": 0x0F&statusbyte}

  def _ready_wait(self):
    """ blocks until ready """
    while not self._check_packet(self._send(b'Q'))["ready"]:
      print("sleep 0.1")
      sleep(0.1)

  def home(self,address = 1):
    """ Homes pump to absolute 0 position """
    self._ready_wait()
    self._send("YR",address)

  def abs_position(self,position, address = 1):
    """ move pump to absolute position position (int)"""
    position = int(position)
    assert position>=0 and position<=3000
    self._ready_wait()
    self._send("A{}R".format(position),address)

  def dispense(self,steps,address=1):
    """ moves syringe up steps (int, 3000=full volume)"""
    steps = int(steps)
    assert steps>=0 and steps<=3000
    self._ready_wait()
    self._send("D{}R".format(steps),address)

  def pickup(self,steps,address=1):
    """ moves syringe down steps (int, 3000=full volume)"""
    steps = int(steps)
    assert steps>=0 and steps<=3000
    self._ready_wait()
    self._send("P{}R".format(steps),address)

  def set_valve(self,position,address=1):
    """ pick valve position: "input" => right, "output" => left """
    self._ready_wait()
    position = position.lower()
    if position.startswith("i"):
      self._send("IR",address)
    elif position.startswith("o"):
      self._send("OR",address)

  def set_speed(self,speed=None,address=1):
    """ sets speed of pump to speed levels 1-40
        speed := 1 => fastest 1.2s per sstroke
        speed := 40 => slowest 600s per stroke
        """
    if speed == None:
      speed = self.default_speed
    speed = int(speed)
    if speed < 1 or speed > 40:
      raise ValueError("speed must be between 1 and 40 (inclusive)")
    print(1)
    self._ready_wait() #make sure this isn't sent while moving
    print(2)
    self._send("S{}R".format(int(speed)),address)
    print(3)
    self.speed = speed

  def mix(self,vol,n=3,speed=10,address=1):
    old_speed = self.speed
    self.set_speed(speed,address)
    for _ in range(n):
      self.pickup(vol)
      self._ready_wait()
      sleep(1)
      self.dispense(vol)
      self._ready_wait()
      sleep(1)

    self.set_speed(old_speed,address)

  def set_aux(self,aux_val,address=1):
    """
      aux_val (int): numerical value of msb-[aux3,aux2,aux1]-lsb
    """
    self._send("J{}R".format(int(aux_val)&0x07))


if __name__ == '__main__':
  p = PSD8()
  if 0:
    p.home()

    p.abs_position(1000)
    p.set_valve("o")
  else:
    #p.mix(100)
    p.abs_position(500)
    p.set_aux(0)

