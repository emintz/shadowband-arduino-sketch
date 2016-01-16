#include "AsyncWrite.h"

/*     
    Example use of overlapped serial output using the AsyncWrite class
    Copyright (C) 2016 The Winer Observeratory, http://www.winer.org

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 */
static const char* message = "Hello, world!\n";
static const uint8_t message_length = 14;


static AsyncWrite asyncWriter;

void setup() {
  // put your setup code here, to run once:
  asyncWriter.init(9600);
}

void loop() {
  if (asyncWriter.process()) {
    asyncWriter.send(message_length, message);
  }
}

