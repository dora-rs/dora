//GUI RENDERING FUNCTIONS:

use tui::{
  layout::{Constraint, Direction, Layout, Rect},
  style::{Color, Style},
  text::{Span, Spans},
  widgets::{Block, Borders, Paragraph, Wrap},
};

/*
pub fn new<T>(items: T) -> List<'a>
where
  T: Into<Vec<ListItem<'a>>>,

pub fn create_list_from_list_items(list_items : Vec<ListItem>, title :  String)
-> List<T>
{
  static highlight : String = ">>".to_string();
  static l : List = List::new(list_items)
    .block(Block::default().title(title).borders(Borders::ALL))
    .style(Style::default().fg(Color::White))
    .highlight_style(Style::default().add_modifier(Modifier::ITALIC))
    .highlight_symbol(&highlight);
    l
}

*/

pub fn create_layput_row(area: Rect) -> Vec<Rect> {
  Layout::default()
    .direction(Direction::Horizontal)
    .constraints([Constraint::Percentage(50), Constraint::Percentage(50)].as_ref())
    .split(area)
}

pub fn create_paragraph_from_string_list(
  strings: Vec<String>,
  title: String,
) -> Paragraph<'static> {
  create_paragraph_form_spans_list(create_spans_list_from_string_list(strings), title)
}

fn create_paragraph_form_spans_list(
  spans: Vec<Spans<'static>>,
  title: String,
) -> Paragraph<'static> {
  Paragraph::new(spans)
    .block(Block::default().title(title).borders(Borders::ALL))
    .style(Style::default().fg(Color::White).bg(Color::Black))
    .wrap(Wrap { trim: true })
}

fn create_spans_list_from_string_list(strings: Vec<String>) -> Vec<Spans<'static>> {
  create_spans_list_from_span_list(create_span_list_from_string_list(strings))
}

fn create_spans_list_from_span_list(span_list: Vec<Span<'static>>) -> Vec<Spans<'static>> {
  let mut spans_list = vec![];
  for span in span_list {
    spans_list.push(Spans::from(span));
  }
  spans_list
}

fn create_span_list_from_string_list(strings: Vec<String>) -> Vec<Span<'static>> {
  let mut span_list = vec![];
  for s in strings {
    span_list.push(Span::raw(s))
  }
  span_list
}
