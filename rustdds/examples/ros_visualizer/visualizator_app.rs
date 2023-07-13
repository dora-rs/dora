use std::time::Duration;

use mio_extras::channel as mio_channel;
use rustdds::{
  discovery::DiscoveredTopicData,
  ros2::builtin_datatypes::{NodeInfo, ROSParticipantInfo},
};
use crossterm::event::{self, Event, KeyCode};
use tui::{
  backend::Backend,
  layout::{Constraint, Direction, Layout},
  style::{Color, Modifier, Style},
  text::{Span, Spans},
  widgets::{Block, Borders, List, ListItem, Tabs},
  Frame,
};

use crate::{
  display_string::{
    get_node_list_strings, get_node_view_strings, get_participant_list_view_strings,
    get_participant_view_strings, get_topic_view_strings, get_topics_list_view_strings,
  },
  messages::DataUpdate,
  stateful_list::StatefulList,
  visualization_helpers::{create_layput_row, create_paragraph_from_string_list},
};

// This handles GUI
// Contains lists of discovered ROS datas.
pub struct VisualizatorApp<'a> {
  pub receiver: mio_channel::Receiver<DataUpdate>,
  tab_titles: Vec<&'a str>,
  tab_index: usize,

  // These list contain datas found by separate ROS thread.
  topic_list_items: Vec<DiscoveredTopicData>,
  external_nodes: Vec<NodeInfo>,
  local_nodes: Vec<NodeInfo>,
  ros_participants: Vec<ROSParticipantInfo>,

  // viewed datas:
  topic_list_display_items: StatefulList<ListItem<'a>>,
  external_nodes_display_items: StatefulList<ListItem<'a>>,
  local_nodes_display_items: StatefulList<ListItem<'a>>,
  ros_participants_display_items: StatefulList<ListItem<'a>>,
}

impl<'a> VisualizatorApp<'a> {
  pub fn new(receiver: mio_channel::Receiver<DataUpdate>) -> VisualizatorApp<'a> {
    VisualizatorApp {
      receiver,
      tab_titles: vec!["Participants", "Nodes", "Topics"],
      tab_index: 0,

      external_nodes: vec![],
      topic_list_items: vec![],
      local_nodes: vec![],
      ros_participants: vec![],
      topic_list_display_items: StatefulList::with_items(vec![]),
      external_nodes_display_items: StatefulList::with_items(vec![]),
      local_nodes_display_items: StatefulList::with_items(vec![]),
      ros_participants_display_items: StatefulList::with_items(vec![]),
    }
  }

  pub fn next_tab(&mut self) {
    self.tab_index = (self.tab_index + 1) % self.tab_titles.len();
  }

  pub fn previous_tab(&mut self) {
    if self.tab_index > 0 {
      self.tab_index -= 1;
    } else {
      self.tab_index = self.tab_titles.len() - 1;
    }
  }

  pub fn handle_user_input(&mut self, timeout: &Duration) -> bool {
    let mut quit_application = false;
    if crossterm::event::poll(*timeout).unwrap() {
      if let Event::Key(key) = event::read().unwrap() {
        match key.code {
          KeyCode::Char('q') => quit_application = true,
          KeyCode::Right => self.next_tab(),
          KeyCode::Left => self.previous_tab(),
          KeyCode::Up => {
            match self.tab_index {
              0 => self.ros_participants_display_items.previous(),
              1 => self.local_nodes_display_items.previous(),
              // 2 => {self.external_nodes_display_items.previous()},
              2 => self.topic_list_display_items.previous(),
              _ => {}
            }
          }
          KeyCode::Down => {
            match self.tab_index {
              0 => self.ros_participants_display_items.next(),
              1 => self.local_nodes_display_items.next(),
              // 2 => {self.external_nodes_display_items.next()},
              2 => self.topic_list_display_items.next(),
              _ => {}
            }
          }
          _ => {}
        }
      }
    }
    quit_application
  }

  pub fn ui<B: Backend>(&mut self, f: &mut Frame<B>) {
    let size = f.size();
    let chunks = Layout::default()
      .direction(Direction::Vertical)
      .margin(1)
      .constraints(
        [
          Constraint::Length(5),
          Constraint::Length(3),
          Constraint::Length(10),
        ]
        .as_ref(),
      )
      .split(size);

    let block = Block::default().style(Style::default().bg(Color::Black).fg(Color::White));
    f.render_widget(block, size);

    let help_strings = vec![
      "Change tab with arrow left and arrow right buttons.".to_string(),
      "Change selected item with arrow up and arrow down buttons".to_string(),
      "Quit application with 'q' button.".to_string(),
    ];
    let user_help =
      create_paragraph_from_string_list(help_strings, " Usage Instructions".to_string());

    f.render_widget(user_help, chunks[0]);

    let titles = self
      .tab_titles
      .iter()
      .map(|t| {
        let (first, rest) = t.split_at(1);
        Spans::from(vec![
          Span::styled(first, Style::default().fg(Color::Yellow)),
          Span::styled(rest, Style::default().fg(Color::Green)),
        ])
      })
      .collect();
    let tabs = Tabs::new(titles)
      .block(Block::default().borders(Borders::ALL).title("Tabs"))
      .select(self.tab_index)
      .style(Style::default().fg(Color::Cyan))
      .highlight_style(
        Style::default()
          .add_modifier(Modifier::BOLD)
          .bg(Color::Black),
      );

    f.render_widget(tabs, chunks[1]);

    let first_row = create_layput_row(chunks[2]);

    let list_of_topics = List::new(self.topic_list_display_items.items.clone())
      .block(Block::default().title("Topics").borders(Borders::ALL))
      .style(Style::default().fg(Color::White))
      .highlight_style(Style::default().add_modifier(Modifier::ITALIC))
      .highlight_symbol(">>");

    let list_of_local_nodes = List::new(self.local_nodes_display_items.items.clone())
      .block(Block::default().title("Nodes").borders(Borders::ALL))
      .style(Style::default().fg(Color::White))
      .highlight_style(Style::default().add_modifier(Modifier::ITALIC))
      .highlight_symbol(">>");

    let _list_of_external_nodes = List::new(self.external_nodes_display_items.items.clone())
      .block(Block::default().title("Nodes").borders(Borders::ALL))
      .style(Style::default().fg(Color::White))
      .highlight_style(Style::default().add_modifier(Modifier::ITALIC))
      .highlight_symbol(">>");

    let list_of_participants = List::new(self.ros_participants_display_items.items.clone())
      .block(Block::default().title("Participants").borders(Borders::ALL))
      .style(Style::default().fg(Color::White))
      .highlight_style(Style::default().add_modifier(Modifier::ITALIC))
      .highlight_symbol(">>");

    let selected_topic_paragraph = create_paragraph_from_string_list(
      self.get_selected_topic_strings(),
      "Topic information".to_string(),
    );
    let selected_participant_paragraph = create_paragraph_from_string_list(
      self.get_selected_participant_strings(),
      "Participant information".to_string(),
    );
    let selected_local_node_paragraph = create_paragraph_from_string_list(
      self.get_selected_local_node_strings(),
      "Node information".to_string(),
    );
    let _selected_external_node_paragraph = create_paragraph_from_string_list(
      self.get_selected_external_node_strings(),
      "Node information".to_string(),
    );

    match self.tab_index {
      0 => {
        f.render_stateful_widget(
          list_of_participants,
          first_row[0],
          &mut self.ros_participants_display_items.state,
        );
        f.render_widget(selected_participant_paragraph, first_row[1]);
      }
      1 => {
        f.render_stateful_widget(
          list_of_local_nodes,
          first_row[0],
          &mut self.local_nodes_display_items.state,
        );
        f.render_widget(selected_local_node_paragraph, first_row[1]);
      }
      /*
      2 => {
        f.render_stateful_widget(list_of_external_nodes, first_row[0], &mut self.external_nodes_display_items.state);
        f.render_widget(selected_external_node_paragraph, first_row[1]);
      },
      */
      2 => {
        f.render_stateful_widget(
          list_of_topics,
          first_row[0],
          &mut self.topic_list_display_items.state,
        );
        f.render_widget(selected_topic_paragraph, first_row[1]);
      }
      4 => {}
      _ => unreachable!(),
    };
  }

  pub fn set_discovered_topics(&mut self, topics: Vec<DiscoveredTopicData>) {
    self.topic_list_items = topics;
    self.set_topics();
  }

  fn set_topics(&mut self) {
    let topic_strings = get_topics_list_view_strings(&self.topic_list_items);
    let previous_state = self.topic_list_display_items.state.clone();
    self.topic_list_display_items = StatefulList::with_items(vec![]);
    for string in topic_strings {
      self.topic_list_display_items.push(ListItem::new(string));
    }
    self.topic_list_display_items.state = previous_state;
  }

  pub fn add_new_ros_participant(&mut self, participant: ROSParticipantInfo) {
    self.ros_participants.push(participant);
    self.set_ros_participants();
  }

  fn set_ros_participants(&mut self) {
    let previous_state = self.ros_participants_display_items.state.clone();

    self.ros_participants_display_items = StatefulList::with_items(vec![]);
    let display_strings = get_participant_list_view_strings(&self.ros_participants);
    for string in display_strings {
      self
        .ros_participants_display_items
        .push(ListItem::new(string))
    }
    self.ros_participants_display_items.state = previous_state;
  }

  pub fn set_node_infos(&mut self, nodes: Vec<NodeInfo>) {
    self.local_nodes = nodes;
    self.set_nodes();
  }

  fn set_nodes(&mut self) {
    let previous_state = self.local_nodes_display_items.state.clone();

    self.local_nodes_display_items = StatefulList::with_items(vec![]);
    let display_strings = get_node_list_strings(&self.local_nodes);
    for string in display_strings {
      self.local_nodes_display_items.push(ListItem::new(string))
    }
    self.local_nodes_display_items.state = previous_state;
  }

  pub fn get_selected_participant_strings(&self) -> Vec<String> {
    self
      .ros_participants_display_items
      .state
      .selected()
      .and_then(|index| self.ros_participants.get(index))
      .map(get_participant_view_strings)
      .unwrap_or_default()
  }

  pub fn get_selected_topic_strings(&self) -> Vec<String> {
    self
      .topic_list_display_items
      .state
      .selected()
      .and_then(|index| self.topic_list_items.get(index))
      .map(get_topic_view_strings)
      .unwrap_or_default()
  }

  pub fn get_selected_local_node_strings(&self) -> Vec<String> {
    self
      .local_nodes_display_items
      .state
      .selected()
      .and_then(|index| self.external_nodes.get(index))
      .map(get_node_view_strings)
      .unwrap_or_default()
  }

  pub fn get_selected_external_node_strings(&self) -> Vec<String> {
    self
      .external_nodes_display_items
      .state
      .selected()
      .and_then(|index| self.external_nodes.get(index))
      .map(get_node_view_strings)
      .unwrap_or_default()
  }
}
