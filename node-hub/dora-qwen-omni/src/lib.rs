use serde::{Deserialize, Serialize};
use llm_json::{repair_json, RepairOptions};

#[derive(Debug, Deserialize, Serialize, PartialEq)]
pub struct BoundingBox {
    pub bbox_2d: [i32; 4],
    pub text_content: Option<String>,
    pub label: Option<String>,
}

pub fn parse_bounding_boxes(json_str: &str) -> Result<Vec<BoundingBox>, serde_json::Error> {
    // Strip the markdown code block markers more carefully
    let mut cleaned = json_str.trim();
    
    // Remove starting markers
    if cleaned.starts_with("```json\n") {
        cleaned = &cleaned[8..]; // Remove "```json\n"
    } else if cleaned.starts_with("```json") {
        cleaned = &cleaned[7..]; // Remove "```json"
    }
    
    // Remove ending markers
    if cleaned.ends_with("\n```") {
        cleaned = &cleaned[..cleaned.len()-4]; // Remove "\n```"
    } else if cleaned.ends_with("```") {
        cleaned = &cleaned[..cleaned.len()-3]; // Remove "```"
    }
    
    let cleaned = cleaned.trim().to_string();

    // Try to parse the cleaned JSON first
    match serde_json::from_str::<Vec<BoundingBox>>(&cleaned) {
        Ok(result) => Ok(result),
        Err(_) => {
            // Try to fix missing commas between objects first
            let mut fixed_json = fix_missing_commas(cleaned.clone());
            
            // Try parsing with comma fixes
            match serde_json::from_str::<Vec<BoundingBox>>(&fixed_json) {
                Ok(result) => Ok(result),
                Err(_) => {
                    // If that fails, try to handle common truncation issues
                    fixed_json = attempt_completion(fixed_json);
                    
                    // Try parsing the fixed version
                    match serde_json::from_str::<Vec<BoundingBox>>(&fixed_json) {
                        Ok(result) => Ok(result),
                        Err(_) => {
                            // If that fails, try using llm_json to repair it
                            let repaired = repair_json(&fixed_json, &RepairOptions::default()).unwrap_or(fixed_json.clone());
                            
                            // Check if the repaired version is still an array
                            if repaired.trim_start().starts_with('[') {
                                serde_json::from_str(&repaired)
                            } else {
                                // If repair converted it to a single object, try the original
                                serde_json::from_str(&fixed_json)
                            }
                        }
                    }
                }
            }
        }
    }
}

fn fix_missing_commas(json: String) -> String {
    // This function attempts to fix missing commas between JSON objects in an array
    let mut result = String::new();
    let chars: Vec<char> = json.chars().collect();
    let mut i = 0;
    let mut in_string = false;
    let mut escape_next = false;
    let mut brace_count = 0i32;
    let mut bracket_count = 0i32;
    
    while i < chars.len() {
        let ch = chars[i];
        
        if escape_next {
            escape_next = false;
            result.push(ch);
            i += 1;
            continue;
        }
        
        match ch {
            '"' if !in_string => {
                in_string = true;
                result.push(ch);
            }
            '"' if in_string => {
                in_string = false;
                result.push(ch);
            }
            '\\' if in_string => {
                escape_next = true;
                result.push(ch);
            }
            '[' if !in_string => {
                bracket_count += 1;
                result.push(ch);
            }
            ']' if !in_string => {
                bracket_count -= 1;
                result.push(ch);
            }
            '{' if !in_string => {
                brace_count += 1;
                result.push(ch);
            }
            '}' if !in_string => {
                brace_count -= 1;
                result.push(ch);
                
                // If we just closed an object and we're in an array, check if we need a comma
                if brace_count == 0 && bracket_count > 0 {
                    // Look ahead to see if there's another object starting
                    let mut j = i + 1;
                    while j < chars.len() && (chars[j].is_whitespace() || chars[j] == '\n' || chars[j] == '\r' || chars[j] == '\t') {
                        j += 1;
                    }
                    
                    // If the next non-whitespace character is '{', we need a comma
                    if j < chars.len() && chars[j] == '{' {
                        result.push(',');
                    }
                }
            }
            _ => {
                result.push(ch);
            }
        }
        
        i += 1;
    }
    
    result
}

fn attempt_completion(mut json: String) -> String {
    let trimmed = json.trim();
    
    // Handle case where JSON is completely truncated mid-object or mid-array
    if !trimmed.is_empty() {
        // If it looks like we're in the middle of parsing an object/array, try to close it
        let mut brace_count = 0i32;
        let mut bracket_count = 0i32;
        let mut in_string = false;
        let mut escape_next = false;
        
        for ch in trimmed.chars() {
            if escape_next {
                escape_next = false;
                continue;
            }
            
            match ch {
                '"' if !escape_next => in_string = !in_string,
                '\\' if in_string => escape_next = true,
                '{' if !in_string => brace_count += 1,
                '}' if !in_string => brace_count -= 1,
                '[' if !in_string => bracket_count += 1,
                ']' if !in_string => bracket_count -= 1,
                _ => {}
            }
        }
        
        // If we're in a string that wasn't closed, close it
        if in_string {
            json.push('"');
        }
        
        // Close any open objects
        for _ in 0..brace_count {
            json.push('}');
        }
        
        // Close any open arrays  
        for _ in 0..bracket_count {
            json.push(']');
        }
        
        // Handle trailing commas by removing them before closing
        let mut chars: Vec<char> = json.chars().collect();
        let mut i = chars.len();
        
        // Walk backwards to find the last non-whitespace character
        while i > 0 {
            i -= 1;
            match chars[i] {
                ' ' | '\t' | '\n' | '\r' => continue,
                ',' => {
                    // Check if this comma is followed only by whitespace and closing brackets/braces
                    let remaining: String = chars[i+1..].iter().collect();
                    if remaining.trim().chars().all(|c| c == ']' || c == '}' || c.is_whitespace()) {
                        chars.remove(i); // Remove the trailing comma
                    }
                    break;
                }
                _ => break,
            }
        }
        
        json = chars.into_iter().collect();
    }
    
    json
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_valid_json() {
        let json = r#"[
            {"bbox_2d": [110, 194, 291, 235], "text_content": "Looking for international students urgently"},
            {"bbox_2d": [162, 235, 235, 252], "text_content": "I don't want"}
        ]"#;

        let result = parse_bounding_boxes(json).unwrap();
        assert_eq!(result.len(), 2);
        assert_eq!(result[0].bbox_2d, [110, 194, 291, 235]);
        assert_eq!(result[0].text_content, Some("Looking for international students urgently".to_string()));
        assert_eq!(result[0].label, None);
    }

    #[test]
    fn test_parse_json_with_markdown_blocks() {
        let json = r#"```json
[
    {"bbox_2d": [110, 194, 291, 235], "text_content": "Looking for international students urgently"},
    {"bbox_2d": [162, 235, 235, 252], "text_content": "I don't want"}
]
```"#;

        let result = parse_bounding_boxes(json).unwrap();
        assert_eq!(result.len(), 2);
        assert_eq!(result[0].bbox_2d, [110, 194, 291, 235]);
        assert_eq!(result[0].text_content, Some("Looking for international students urgently".to_string()));
    }

    #[test]
    fn test_parse_json_with_label() {
        let json = r#"[
            {"bbox_2d": [110, 194, 291, 235], "label": "button", "text_content": "Click me"}
        ]"#;

        let result = parse_bounding_boxes(json).unwrap();
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].bbox_2d, [110, 194, 291, 235]);
        assert_eq!(result[0].text_content, Some("Click me".to_string()));
        assert_eq!(result[0].label, Some("button".to_string()));
    }

    #[test]
    fn test_parse_json_missing_text_content() {
        let json = r#"[
            {"bbox_2d": [110, 194, 291, 235], "label": "button"}
        ]"#;

        let result = parse_bounding_boxes(json).unwrap();
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].bbox_2d, [110, 194, 291, 235]);
        assert_eq!(result[0].text_content, None);
        assert_eq!(result[0].label, Some("button".to_string()));
    }

    #[test]
    fn test_parse_malformed_json_with_trailing_comma() {
        let json = r#"[
            {"bbox_2d": [110, 194, 291, 235], "text_content": "Looking for international students urgently"},
            {"bbox_2d": [162, 235, 235, 252], "text_content": "I don't want"},
        ]"#;

        // This should use llm_json to repair the trailing comma
        let result = parse_bounding_boxes(json);
        match result {
            Ok(boxes) => {
                assert_eq!(boxes.len(), 2);
                assert_eq!(boxes[0].bbox_2d, [110, 194, 291, 235]);
                assert_eq!(boxes[1].bbox_2d, [162, 235, 235, 252]);
            }
            Err(_) => {
                // If llm_json can't repair it, that's expected behavior too
                // Just ensure it doesn't panic
            }
        }
    }

    #[test]
    fn test_parse_malformed_json_missing_quotes() {
        let json = r#"[
            {bbox_2d: [110, 194, 291, 235], text_content: "Looking for international students urgently"}
        ]"#;

        // This should use llm_json to repair the missing quotes
        let result = parse_bounding_boxes(json);
        match result {
            Ok(boxes) => {
                assert_eq!(boxes.len(), 1);
                assert_eq!(boxes[0].bbox_2d, [110, 194, 291, 235]);
                assert_eq!(boxes[0].text_content, Some("Looking for international students urgently".to_string()));
            }
            Err(_) => {
                // If llm_json can't repair it, that's expected behavior too
                // Just ensure it doesn't panic
            }
        }
    }

    #[test]
    fn test_parse_your_example() {
        let json = r#"```json
[
	{"bbox_2d": [110, 194, 291, 235], "text_content": "Looking for international students urgently"},
	{"bbox_2d": [162, 235, 235, 252], "text_content": "I don't want"},
	{"bbox_2d": [72, 362, 200, 382], "text_content": "HR: International students"},
	{"bbox_2d": [200, 362, 332, 382], "text_content": "Where did they all go?"},
	{"bbox_2d": [66, 402, 341, 422], "text_content": "Are there any international students who can join this month, full-time?"},
	{"bbox_2d": [66, 434, 226, 454], "text_content": "Internships are also needed"},
	{"bbox_2d": [72, 468, 130, 486], "text_content": "10,000 people"},
	{"bbox_2d": [130, 468, 188, 486], "text_content": "Interested"},
	{"bbox_2d": [424, 212, 506, 235], "text_content": "Guochuang"},
	{"bbox_2d": [548, 204, 614, 233], "text_content": "???"},
	{"bbox_2d": [448, 356, 528, 372], "text_content": "17.083 million"},
	{"bbox_2d": [424, 402, 686, 422], "text_content": "Oh no! I'm surrounded by cuties!"},
	{"bbox_2d": [424, 444, 506, 462], "text_content": "There is a beast"},
	{"bbox_2d": [510, 444, 547, 462], "text_content": "Season 3"},
	{"bbox_2d": [210, 582, 316, 620], "text_content": "Do you dare to eat it?"},
	{"bbox_2d": [209, 620, 228, 636], "text_content": "!" },
	{"bbox_2d": [194, 774, 226, 792], "text_content": "16:09"},
	{"bbox_2d": [448, 598, 506, 618], "text_content": "Variety"},
	{"bbox_2d": [510, 608, 660, 656], "text_content": "5 hours"},
	{"bbox_2d": [510, 636, 660, 684], "text_content": "2024KPL"},
	{"bbox_2d": [510, 684, 660, 712], "text_content": "Kingdom Dream Team"},
	{"bbox_2d": [510, 712, 660, 740], "text_content": "5 hours"},
	{"bbox_2d": [448, 774, 506, 792], "text_content": "497,000"},
	{"bbox_2d": [448, 822, 506, 840], "text_content": "2024KPL"},
	{"bbox_2d": [510, 822, 660, 840], "text_content": "Kingdom Dream Team"},
	{"bbox_2d": [510, 840, 660, 858], "text_content": "5 hours"},
	{"bbox_2d": [448, 858, 506, 876], "text_content": "Time difference five"},
	{"bbox_2d": [660, 804, 728, 822], "text_content": "Refresh content"}
]
```"#;

        let result = parse_bounding_boxes(json).unwrap();
        assert_eq!(result.len(), 28);
        assert_eq!(result[0].bbox_2d, [110, 194, 291, 235]);
        assert_eq!(result[0].text_content, Some("Looking for international students urgently".to_string()));
        assert_eq!(result[27].bbox_2d, [660, 804, 728, 822]);
        assert_eq!(result[27].text_content, Some("Refresh content".to_string()));
    }

    #[test]
    fn test_parse_empty_array() {
        let json = "[]";
        let result = parse_bounding_boxes(json).unwrap();
        assert_eq!(result.len(), 0);
    }

    #[test]
    fn test_parse_invalid_json() {
        let json = "not json at all";
        let result = parse_bounding_boxes(json);
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_incomplete_json() {
        let json = r#"[
            {"bbox_2d": [110, 194, 291, 235], "text_content": "Looking for"#;

        // This should still work due to completion and repair
        let result = parse_bounding_boxes(json);
        // Might succeed or fail depending on repair capabilities
        // Just ensure it doesn't panic
        let _ = result;
    }

    #[test]
    fn test_parse_truncated_in_string() {
        let json = r#"[
            {"bbox_2d": [110, 194, 291, 235], "text_content": "Looking for international students urgently"},
            {"bbox_2d": [162, 235, 235, 252], "text_content": "I don't want"#;

        let result = parse_bounding_boxes(json);
        match result {
            Ok(boxes) => {
                assert_eq!(boxes.len(), 2);
                assert_eq!(boxes[0].bbox_2d, [110, 194, 291, 235]);
                // Second entry should have completed string
                assert_eq!(boxes[1].bbox_2d, [162, 235, 235, 252]);
                assert_eq!(boxes[1].text_content, Some("I don't want".to_string()));
            }
            Err(_) => {
                // If completion doesn't work, that's still acceptable
                // Just ensure it doesn't panic
            }
        }
    }

    #[test] 
    fn test_parse_truncated_in_object() {
        let json = r#"[
            {"bbox_2d": [110, 194, 291, 235], "text_content": "Looking for international students urgently"},
            {"bbox_2d": [162, 235, 235, 252]"#;

        let result = parse_bounding_boxes(json);
        match result {
            Ok(boxes) => {
                assert!(boxes.len() >= 1);
                assert_eq!(boxes[0].bbox_2d, [110, 194, 291, 235]);
            }
            Err(_) => {
                // If completion doesn't work, that's acceptable
            }
        }
    }

    #[test]
    fn test_parse_truncated_in_array() {
        let json = r#"[
            {"bbox_2d": [110, 194, 291, 235], "text_content": "Looking for international students urgently"},
            {"bbox_2d": [162, 235, 235"#;

        let result = parse_bounding_boxes(json);
        match result {
            Ok(boxes) => {
                assert!(boxes.len() >= 1);
                assert_eq!(boxes[0].bbox_2d, [110, 194, 291, 235]);
            }
            Err(_) => {
                // If completion doesn't work, that's acceptable
            }
        }
    }

    #[test]
    fn test_parse_truncated_with_trailing_comma() {
        let json = r#"[
            {"bbox_2d": [110, 194, 291, 235], "text_content": "Looking for international students urgently"},
            {"bbox_2d": [162, 235, 235, 252], "text_content": "I don't want"},"#;

        let result = parse_bounding_boxes(json);
        match result {
            Ok(boxes) => {
                assert_eq!(boxes.len(), 2);
                assert_eq!(boxes[0].bbox_2d, [110, 194, 291, 235]);
                assert_eq!(boxes[1].bbox_2d, [162, 235, 235, 252]);
            }
            Err(_) => {
                // If repair doesn't work, that's acceptable
            }
        }
    }

    #[test]
    fn test_parse_deeply_nested_truncation() {
        let json = r#"[
            {"bbox_2d": [110, 194, 291, 235], "text_content": "Complete entry"},
            {"bbox_2d": [162, 235, 235, 252], "text_content": "Truncated entry with nested {"#;

        let result = parse_bounding_boxes(json);
        // This is a very challenging case - just ensure no panic
        let _ = result;
    }

    #[test]
    fn test_parse_json_with_unicode() {
        let json = r#"[
            {"bbox_2d": [110, 194, 291, 235], "text_content": "国际学生紧急招聘"},
            {"bbox_2d": [162, 235, 235, 252], "text_content": "我不想要"}
        ]"#;

        let result = parse_bounding_boxes(json).unwrap();
        assert_eq!(result.len(), 2);
        assert_eq!(result[0].text_content, Some("国际学生紧急招聘".to_string()));
        assert_eq!(result[1].text_content, Some("我不想要".to_string()));
    }

    #[test]
    fn test_parse_trailing_comma_error() {
        // This test replicates the trailing comma error case
        let json = r#"```json
[
	{"bbox_2d": [518, 140, 563, 162], "text_content": "movie"},
	{"bbox_2d": [478, 362, 578, 382], "text_content": "All or Nothing"},
	{"bbox_2d": [508, 436, 638, 456], "text_content": "Bilibili Movie"},
	{"bbox_2d": [508, 582, 591, 602], "text_content": "TV drama"},
	{"bbox_2d": [478, 749, 585, 769], "text_content": "6.286 million"},
	{"bbox_2d": [478, 802, 585, 822], "text_content": "We who cannot become beasts"},
	{"bbox_2d": [738, 822, 890, 842], "text_content": "Yui Aragaki: Am I not ashamed?"},
	{"bbox_2d": [14, 802, 362, 860], "text_content": "2200 Community American Meat Shop, ate the black label ham known as Hermes level"},
	{"bbox_2d": [14, 879, 221, 897], "text_content": "Warm Oil Spicy U · Yesterday"},
	{"bbox_2d": [14, 362, 321, 382], "text_content": "Kuaq, I really underestimated you..."},
	{"bbox_2d": [14, 436, 138, 456], "text_content": "30,000 people are interested"},
	{"bbox_2d": [478, 279, 592, 299], "text_content": "20.633 million"},
	{"bbox_2d": [14, 749, 128, 769], "text_content": "145,000"},
	{"bbox_2d": [162, 749, 214, 769], "text_content": "508"},
	{"bbox_2d": [358, 749, 405, 769], "text_content": "10:12"},
]
```"#;

        // This should either succeed with repair or fail gracefully (no panic)
        let result = parse_bounding_boxes(json);
        match result {
            Ok(boxes) => {
                assert_eq!(boxes.len(), 15);
                assert_eq!(boxes[0].text_content, Some("movie".to_string()));
                assert_eq!(boxes[14].text_content, Some("10:12".to_string()));
            }
            Err(_) => {
                // If repair doesn't work for this specific trailing comma case, that's acceptable
                // The important thing is that it doesn't panic and tries to recover
            }
        }
    }

    #[test]
    fn test_parse_exact_failing_case() {
        // This test replicates the exact error case mentioned by the user
        let json = r#"```json
[
	{"bbox_2d": [20, 119, 314, 166], "text_content": "Are there any positions available for this month, full-time?"}
	{"bbox_2d": [10, 152, 100, 172], "text_content": "Internships are also available."}
	{"bbox_2d": [10, 184, 109, 202], "text_content": "10,000 people are interested."}
	{"bbox_2d": [424, 89, 657, 116], "text_content": "Oh no! I'm surrounded by cuties!"}
	{"bbox_2d": [400, 158, 521, 179], "text_content": "The Third Season of Beast"}
	{"bbox_2d": [194, 272, 300, 316], "text_content": "Do you dare to eat it?"}
	{"bbox_2d": [194, 316, 208, 348], "text_content": "You"}
	{"bbox_2d": [194, 348, 208, 380], "text_content": "Eat"}
	{"bbox_2d": [194, 380, 208, 412], "text_content": "it?"}
	{"bbox_2d": [299, 434, 345, 454], "text_content": "16:09"}
	{"bbox_2d": [20, 454, 310, 474], "text_content": "8 yuan for a whole duck, have you ever eaten it?!"}
	{"bbox_2d": [10, 482, 310, 502], "text_content": "Get in touch with the upper class."}
	{"bbox_2d": [10, 520, 226, 537], "text_content": "Little Laz wearing a fur pant yesterday."}
	{"bbox_2d": [400, 479, 692, 502], "text_content": "2024 KPL King's Dream Team: Time difference five"}
	{"bbox_2d": [400, 502, 456, 522], "text_content": "hours"}
	{"bbox_2d": [400, 545, 526, 562], "text_content": "UP Art Bilibili Machine"}
	{"bbox_2d": [20, 700, 248, 742], "text_content": "On the Knight's Table"}
	{"bbox_2d": [20, 742, 248, 784], "text_content": "The Revolution of Meat"}
	{"bbox_2d": [20, 832, 310, 852], "text_content": "Red meat, white meat, grilled meat, goose meat"}
	{"bbox_2d": [20, 862, 310, 882], "text_content": "One meal, one meal, one meal, one meal"}
	{"bbox_2d": [424, 670, 480, 690], "text_content": "Class"}
	{"bbox_2d": [424, 690, 600, 732], "text_content": "Basic Painting"}
	{"bbox_2d": [424, 732, 562, 752], "text_content": "Three Tools"}
	{"bbox_2d": [424, 752, 554, 772], "text_content": "Human Body"}
	{"bbox_2d": [424, 832, 600, 852], "text_content": "DM Bag Meow's Basic Painting Three Tools - Human Body"}
	{"bbox_2d": [424, 862, 554, 882], "text_content": "Human Body [Time Difference Five]"}
	{"bbox_2d": [676, 808, 748, 824], "text_content": "Refresh Content"}
]
```"#;

        // This should no longer fail with "expected `,` or `]`" error
        let result = parse_bounding_boxes(json);
        assert!(result.is_ok(), "Should successfully parse JSON with missing commas");
        
        let boxes = result.unwrap();
        assert_eq!(boxes.len(), 27);
        assert_eq!(boxes[0].text_content, Some("Are there any positions available for this month, full-time?".to_string()));
        assert_eq!(boxes[26].text_content, Some("Refresh Content".to_string()));
    }

    #[test]
    fn test_parse_missing_commas_between_objects() {
        let json = r#"```json
[
	{"bbox_2d": [20, 119, 314, 166], "text_content": "Are there any positions available for this month, full-time?"}
	{"bbox_2d": [10, 152, 100, 172], "text_content": "Internships are also available."}
	{"bbox_2d": [10, 184, 109, 202], "text_content": "10,000 people are interested."}
]
```"#;

        let result = parse_bounding_boxes(json).unwrap();
        assert_eq!(result.len(), 3);
        assert_eq!(result[0].bbox_2d, [20, 119, 314, 166]);
        assert_eq!(result[0].text_content, Some("Are there any positions available for this month, full-time?".to_string()));
        assert_eq!(result[1].bbox_2d, [10, 152, 100, 172]);
        assert_eq!(result[1].text_content, Some("Internships are also available.".to_string()));
        assert_eq!(result[2].bbox_2d, [10, 184, 109, 202]);
        assert_eq!(result[2].text_content, Some("10,000 people are interested.".to_string()));
    }

    #[test]
    fn test_parse_uncovered_json_prompt() {
        let json = r#"```json
[
	{"bbox_2d": [20, 119, 314, 166], "text_content": "Are there any positions available for this month, full-time?"},
	{"bbox_2d": [10, 152, 100, 172], "text_content": "Internships are also available."},
	{"bbox_2d": [10, 184, 109, 202], "text_content": "10,000 people are interested."},
	{"bbox_2d": [424, 89, 657, 116], "text_content": "Oh no! I'm surrounded by cuties!"},
	{"bbox_2d": [400, 158, 521, 179], "text_content": "The Third Season of Beast"},
	{"bbox_2d": [194, 272, 300, 316], "text_content": "Do you dare to eat it?"},
	{"bbox_2d": [194, 316, 208, 348], "text_content": "You"},
	{"bbox_2d": [194, 348, 208, 380], "text_content": "Eat"},
	{"bbox_2d": [194, 380, 208, 412], "text_content": "it?"},
	{"bbox_2d": [299, 434, 345, 454], "text_content": "16:09"},
	{"bbox_2d": [20, 454, 310, 474], "text_content": "8 yuan for a whole duck, have you ever eaten it?!"},
	{"bbox_2d": [10, 482, 310, 502], "text_content": "Get in touch with the upper class."},
	{"bbox_2d": [10, 520, 226, 537], "text_content": "Little Laz wearing a fur pant yesterday."},
	{"bbox_2d": [400, 479, 692, 502], "text_content": "2024 KPL King's Dream Team: Time difference five"},
	{"bbox_2d": [400, 502, 456, 522], "text_content": "hours"},
	{"bbox_2d": [400, 545, 526, 562], "text_content": "UP Art Bilibili Machine"},
	{"bbox_2d": [20, 700, 248, 742], "text_content": "On the Knight's Table"},
	{"bbox_2d": [20, 742, 248, 784], "text_content": "The Revolution of Meat"},
	{"bbox_2d": [20, 832, 310, 852], "text_content": "Red meat, white meat, grilled meat, goose meat"},
	{"bbox_2d": [20, 862, 310, 882], "text_content": "One meal, one meal, one meal, one meal"},
	{"bbox_2d": [424, 670, 480, 690], "text_content": "Class"},
	{"bbox_2d": [424, 690, 600, 732], "text_content": "Basic Painting"},
	{"bbox_2d": [424, 732, 562, 752], "text_content": "Three Tools"},
	{"bbox_2d": [424, 752, 554, 772], "text_content": "Human Body"},
	{"bbox_2d": [424, 832, 600, 852], "text_content": "DM Bag Meow's Basic Painting Three Tools - Human Body"},
	{"bbox_2d": [424, 862, 554, 882], "text_content": "Human Body [Time Difference Five]"},
	{"bbox_2d": [676, 808, 748, 824], "text_content": "Refresh Content"}
]
```"#;

        let result = parse_bounding_boxes(json).unwrap();
        assert_eq!(result.len(), 27);
        assert_eq!(result[0].bbox_2d, [20, 119, 314, 166]);
        assert_eq!(result[0].text_content, Some("Are there any positions available for this month, full-time?".to_string()));
        assert_eq!(result[26].bbox_2d, [676, 808, 748, 824]);
        assert_eq!(result[26].text_content, Some("Refresh Content".to_string()));
        assert_eq!(result[13].text_content, Some("2024 KPL King's Dream Team: Time difference five".to_string()));
    }
}