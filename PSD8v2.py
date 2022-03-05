from time import sleep
import serial
import os

"""
Changes to enable running in python3 env:
converted strings to bytes before manipulating them
in particular in _send. Changed checksum to operate
on bytes rather than string characters and to
return a bytes object of length 1.

Also parsing error codes now using normal strings as
opposed to opening the file in bytes mode.
"""

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
    with open(pth+"/"+"error_codes.csv",'r') as codes:
      for line in codes:
        toks = line.split(',')
        key = int(toks[0])
        error = toks[1].strip()
        self.error_codes[key] = error
    self.set_speed(self.default_speed)

  @staticmethod
  def _checksum(s):
    """ returns checksum as int """
    if isinstance(s, str):
      s = s.encode()
    res = 0
    for b in s:
      res ^= b
      res &= 0xFF
    return res.to_bytes(1, 'big')
    # return reduce(lambda x,y:x^(ord(y)), s, 0)

  def _send(self,cmd,address=1):
    """TODO: handle retrys and sequence numbers correctly

    """
    retry_count = 10 #TODO: parameterize?
    repeat = 0 # was unused, mult 0 by num, never used again

    if isinstance(address, str):
      address = address.encode()    #address is not a bytes type

    packet = (
      b"\x02" +
      str(address).encode() +
      # chr( (str(self.sequence).encode()+ord("0") ) | (0b00001000*repeat) ) +
      str(self.sequence).encode() +
      cmd.encode() +
      b"\x03"
    )
    packet += self._checksum(packet)

    while retry_count>0:
      self.spt.reset_input_buffer()
      self.spt.write(packet)
      in_packet = b""

      #how many times should we keep trying for data if we don't see the end?
      read_count = 10
      # note byte before checksum needs to be 3 aka b'\0x3'
      while (len(in_packet) == 0 or in_packet[-2] != 3) and read_count > 0:
        sleep(8.0*len(packet)/self.spt.baudrate+0.01)
        read_count -= 1
        in_packet += self.spt.read(100)

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
    
    #ensure it's addressed to me and complete
    if (not packet.startswith(b"\x020")) or packet[-2] != 3:
      raise ValueError("must be a complete response packet.  got: {}".format(str( map(ord,packet))))
    statusbyte = packet[2]
    #check checksum
    if self._checksum(packet[:-1]) != packet[-1].to_bytes(1, "big"):
      raise ValueError("failed checksum")
    #check constant fields
    if statusbyte&0b11010000 != 0b01000000:
      raise ValueError("status byte invalid")

    return {
      "ready":        bool(0b00100000 & statusbyte),
      "errorStatus":  0x0F&statusbyte
    }

  def _ready_wait(self):
    """ blocks until ready """
    while not self._check_packet(self._send("Q"))["ready"]:
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
    """
    sets speed of pump to speed levels 1-40
    speed := 1 => fastest 1.2s per sstroke
    speed := 40 => slowest 600s per stroke
    """
    if speed == None:
      speed = self.default_speed
    speed = int(speed)
    if speed < 1 or speed > 40:
      raise ValueError("speed must be between 1 and 40 (inclusive)")
    self._ready_wait() #make sure this isn't sent while moving
    self._send("S{}R".format(int(speed)),address)
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

  p.home()

  p.abs_position(1000)
  p.set_valve("0")
  p.pickup(10)
  # else:
  #   #p.mix(100)
  #   p.abs_position(500)
  #   p.set_aux(0)

