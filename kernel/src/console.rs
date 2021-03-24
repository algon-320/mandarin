use crate::graphics::{font::Font, Color, Render};

#[derive(Debug, Clone, Copy)]
pub struct Attribute {
    pub fg: Color,
    pub bg: Color,
}
impl Attribute {
    pub const DEFAULT: Self = Attribute {
        fg: Color::WHITE,
        bg: Color::BLACK,
    };
}

#[derive(Debug, Clone, Default)]
struct Cursor {
    x: usize,
    y: usize,
}
#[derive(Debug, Clone, Copy)]
struct Cell {
    ch: Option<char>,
    attr: Attribute,
}
impl Cell {
    const DEFAULT: Self = Cell {
        ch: None,
        attr: Attribute::DEFAULT,
    };
}
#[derive(Debug)]
pub struct Console {
    columns: usize,
    lines: usize,
    cells: [Cell; 12000],
    cursor: Cursor,
    pub attr: Attribute,
}
impl Console {
    pub const fn new(columns: usize, lines: usize) -> Self {
        Self {
            columns,
            lines,
            cells: [Cell::DEFAULT; 12000],
            cursor: Cursor { x: 0, y: 0 },
            attr: Attribute::DEFAULT,
        }
    }
    pub fn resize(&mut self, columns: usize, lines: usize) {
        // TODO: check the size
        use core::cmp::min;
        self.lines = min(lines, 60);
        self.columns = min(columns, 200);
    }

    fn scroll_up(&mut self) {
        let end = self.lines * self.columns;
        let src = self.columns..end;
        self.cells.copy_within(src, 0);
        self.cells[(end - self.columns)..end].fill(Cell::DEFAULT);
    }

    fn move_cursor_forward(&mut self) {
        self.cursor.x += 1;
        if self.cursor.x == self.columns {
            self.cursor.x = 0;
            if self.cursor.y + 1 == self.lines {
                self.scroll_up();
            } else {
                self.cursor.y += 1;
            }
        }
    }

    fn move_cursor_newline(&mut self) {
        self.cursor.x = 0;
        if self.cursor.y + 1 == self.lines {
            self.scroll_up();
        } else {
            self.cursor.y += 1;
        }
    }

    pub fn insert_char(&mut self, ch: char) {
        match ch {
            '\n' => {
                self.move_cursor_newline();
            }
            ch => {
                let idx = self.cursor.y * self.columns + self.cursor.x;
                self.cells[idx] = Cell {
                    ch: Some(ch),
                    attr: self.attr,
                };
                self.move_cursor_forward();
            }
        }
    }

    pub fn render<R: Render, F: Font>(&self, renderer: &mut R, font: &mut F) {
        let (char_w, char_h) = font.char_size('M');
        for line in 0..self.lines {
            for col in 0..self.columns {
                let idx = line * self.columns + col;
                if let Cell { ch: Some(ch), attr } = self.cells[idx] {
                    let x = char_w * col as isize;
                    let y = char_h * line as isize;
                    renderer.draw_filled_rect(x, y, char_w, char_h, attr.bg);
                    font.draw_char(renderer, x, y, attr.fg, ch);
                }
            }
        }
    }
}

impl core::fmt::Write for Console {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for c in s.chars() {
            self.insert_char(c);
        }
        Ok(())
    }
}

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {{
        use $crate::console::Attribute;
        $crate::global::console_write(Attribute::DEFAULT, format_args!($($arg)*));
    }};
}

#[macro_export]
macro_rules! println {
    () => { $crate::print!("\n") };
    ($($arg:tt)*) => { $crate::print!("{}\n", format_args!($($arg)*)) };
}

#[macro_export]
macro_rules! error {
    ($($arg:tt)*) => {{
        use $crate::console::Attribute;
        let attr = Attribute { fg: Color::RED, bg: Color::BLACK };
        $crate::global::console_write(attr, format_args!($($arg)*));
    }};
}

#[macro_export]
macro_rules! errorln {
    () => { $crate::error!("\n") };
    ($($arg:tt)*) => { $crate::error!("{}\n", format_args!($($arg)*)) };
}
