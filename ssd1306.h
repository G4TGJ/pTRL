void oledWriteString(int col, int row, const char *str, int fontNum, bool render);
void oledInit( void );
void oledDrawLine(int x0, int y0, int x1, int y1, bool on, bool render);
void oledDrawRectangle(int x0, int y0, int width, int height, bool on, bool render);
