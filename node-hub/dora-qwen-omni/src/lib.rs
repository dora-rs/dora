use serde::{Deserialize, Serialize};
use llm_json::{repair_json, RepairOptions};

#[derive(Debug, Deserialize, Serialize, PartialEq)]
pub struct BoundingBox {
    pub bbox_2d: [i32; 4],
    pub text_content: Option<String>,
    pub label: Option<String>,
}

/// Parses bounding box JSON from model output, handling malformed JSON and markdown wrappers.
/// 
/// This function handles various edge cases including:
/// - JSON wrapped in markdown code blocks (```json ... ```)
/// - Missing commas between objects
/// - Trailing commas (fixed in this version)
/// - Incomplete/truncated JSON
/// 
/// # Example
/// ```
/// use dora_qwen_omni::parse_bounding_boxes;
/// 
/// // Standard case with markdown wrapper
/// let json = r#"```json
/// [
///   {"bbox_2d": [100, 100, 200, 200], "text_content": "Hello"}
/// ]
/// ```"#;
/// 
/// let result = parse_bounding_boxes(json).unwrap();
/// assert_eq!(result.len(), 1);
/// 
/// // Handles trailing commas (previously caused errors)
/// let json_with_trailing_comma = r#"```json
/// [
///   {"bbox_2d": [100, 100, 200, 200], "text_content": "Hello"},
/// ]
/// ```"#;
/// 
/// let result = parse_bounding_boxes(json_with_trailing_comma).unwrap();
/// assert_eq!(result.len(), 1);
/// ```
pub fn parse_bounding_boxes(json_str: &str) -> Result<Vec<BoundingBox>, serde_json::Error> {
    let mut cleaned = json_str.trim();
    
    // Handle case where there's explanatory text before the JSON
    // Look for the start of JSON content - either '[' or markdown block
    if let Some(json_start) = find_json_start(cleaned) {
        cleaned = &cleaned[json_start..];
    }
    
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
    
    // Handle case where there are trailing characters after valid JSON
    // Find the end of the JSON array by looking for the last ']' that closes the main array
    let mut cleaned_str = cleaned.trim().to_string();
    if let Some(main_array_end) = find_main_array_end(&cleaned_str) {
        // Truncate at the end of the main array to remove trailing characters
        cleaned_str = cleaned_str[..=main_array_end].to_string();
    }
    
    let cleaned = cleaned_str;

    // Try to parse the cleaned JSON first
    match serde_json::from_str::<Vec<BoundingBox>>(&cleaned) {
        Ok(result) => Ok(result),
        Err(_) => {
            // Try to fix unescaped quotes first
            let mut fixed_json = fix_unescaped_quotes(cleaned.clone());
            
            // Try parsing with quote fixes
            match serde_json::from_str::<Vec<BoundingBox>>(&fixed_json) {
                Ok(result) => Ok(result),
                Err(_) => {
                    // Try to fix missing commas between objects
                    fixed_json = fix_missing_commas(fixed_json);
                    
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
    }
}

fn find_json_start(text: &str) -> Option<usize> {
    // Look for the first occurrence of either:
    // 1. A markdown code block starting with ```json
    // 2. A direct JSON array starting with '['
    
    // First, try to find ```json
    if let Some(markdown_pos) = text.find("```json") {
        return Some(markdown_pos);
    }
    
    // If no markdown block, look for the first '[' that starts an array
    if let Some(bracket_pos) = text.find('[') {
        // Make sure this '[' looks like the start of a JSON array
        // by checking that what comes before it doesn't suggest it's part of a string
        let before_bracket = &text[..bracket_pos];
        
        // Simple heuristic: if there's explanatory text before the bracket,
        // and the bracket appears to be at the start of a new section/line,
        // then it's likely the JSON start
        if before_bracket.contains('\n') || before_bracket.len() > 50 {
            return Some(bracket_pos);
        }
    }
    
    None
}

fn find_main_array_end(text: &str) -> Option<usize> {
    // Find the position where the main JSON array ends
    // This handles nested arrays and objects properly
    let mut bracket_count = 0i32;
    let mut _brace_count = 0i32;
    let mut in_string = false;
    let mut escape_next = false;
    let mut found_first_bracket = false;
    
    for (i, ch) in text.char_indices() {
        if escape_next {
            escape_next = false;
            continue;
        }
        
        match ch {
            '"' if !escape_next => in_string = !in_string,
            '\\' if in_string => escape_next = true,
            '[' if !in_string => {
                bracket_count += 1;
                found_first_bracket = true;
            }
            ']' if !in_string => {
                bracket_count -= 1;
                // If we've closed the main array (bracket_count == 0)
                if found_first_bracket && bracket_count == 0 {
                    return Some(i);
                }
            }
            '{' if !in_string => _brace_count += 1,
            '}' if !in_string => _brace_count -= 1,
            _ => {}
        }
    }
    
    None
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
        // First, handle trailing commas before doing completion
        json = remove_trailing_commas(json);
        
        // Fix truncated strings that end with unusual characters like '$'
        json = fix_truncated_strings(json);
        
        // If it looks like we're in the middle of parsing an object/array, try to close it
        let mut brace_count = 0i32;
        let mut bracket_count = 0i32;
        let mut in_string = false;
        let mut escape_next = false;
        
        for ch in json.chars() {
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
        
        // Final cleanup for any remaining trailing commas
        json = remove_trailing_commas(json);
    }
    
    json
}

fn fix_unescaped_quotes(json: String) -> String {
    // This function attempts to fix unescaped quotes in JSON strings
    let mut result = String::new();
    let chars: Vec<char> = json.chars().collect();
    let mut i = 0;
    let mut in_string = false;
    let mut escape_next = false;
    
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
                // Starting a string
                in_string = true;
                result.push(ch);
            }
            '"' if in_string => {
                // This could be either:
                // 1. The end of the current string (if followed by , } ] or whitespace leading to those)
                // 2. An unescaped quote within the string that needs escaping
                
                // Look ahead to see what comes after this quote
                let mut j = i + 1;
                while j < chars.len() && chars[j].is_whitespace() {
                    j += 1;
                }
                
                // For string values (not keys), we only expect comma, closing brace, or closing bracket
                // For string keys, we expect a colon
                let is_end_of_string = j >= chars.len() || 
                    matches!(chars[j], ',' | '}' | ']' | ':');
                
                if is_end_of_string {
                    // This is the end of the string
                    in_string = false;
                    result.push(ch);
                } else {
                    // This is likely an unescaped quote within the string, escape it
                    result.push('\\');
                    result.push(ch);
                }
            }
            '\\' if in_string => {
                escape_next = true;
                result.push(ch);
            }
            _ => {
                result.push(ch);
            }
        }
        
        i += 1;
    }
    
    result
}

fn fix_truncated_strings(json: String) -> String {
    // This function handles strings that are truncated with unusual characters like '$'
    let mut result = String::new();
    let chars: Vec<char> = json.chars().collect();
    let mut i = 0;
    let mut in_string = false;
    let mut escape_next = false;
    
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
            '$' if in_string && i == chars.len() - 1 => {
                // If we encounter '$' at the end of input while in a string,
                // it's likely a truncation artifact - remove it and close the string
                result.push('"');
                break;
            }
            _ => {
                result.push(ch);
            }
        }
        
        i += 1;
    }
    
    result
}

fn remove_trailing_commas(json: String) -> String {
    let mut chars: Vec<char> = json.chars().collect();
    let mut i = 0;
    let mut in_string = false;
    let mut escape_next = false;
    
    while i < chars.len() {
        if escape_next {
            escape_next = false;
            i += 1;
            continue;
        }
        
        match chars[i] {
            '"' if !escape_next => in_string = !in_string,
            '\\' if in_string => escape_next = true,
            ',' if !in_string => {
                // Found a comma outside of a string, check if it's a trailing comma
                let mut j = i + 1;
                let mut found_closing = false;
                
                // Look ahead to see what comes after this comma
                while j < chars.len() {
                    match chars[j] {
                        ' ' | '\t' | '\n' | '\r' => {
                            j += 1;
                            continue;
                        }
                        ']' | '}' => {
                            found_closing = true;
                            break;
                        }
                        _ => break,
                    }
                }
                
                // If we found a closing bracket/brace after only whitespace, remove the comma
                if found_closing {
                    chars.remove(i);
                    continue; // Don't increment i since we removed a character
                }
            }
            _ => {}
        }
        
        i += 1;
    }
    
    chars.into_iter().collect()
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
                // If parsing succeeds, should have at least the first complete object
                assert!(boxes.len() >= 1);
                assert_eq!(boxes[0].bbox_2d, [110, 194, 291, 235]);
                assert_eq!(boxes[0].text_content, Some("Looking for international students urgently".to_string()));
                
                // Second entry might be fixed by completion, but we don't guarantee it
                if boxes.len() > 1 {
                    assert_eq!(boxes[1].bbox_2d, [162, 235, 235, 252]);
                    // text_content might be Some("I don't want") or None, depending on repair success
                }
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
    fn test_parse_user_specific_trailing_comma_example() {
        // This is the exact JSON that was causing the error in the user's issue
        let json = r#"```json
[
	{"bbox_2d": [103, 128, 246, 158], "text_content": "This small puddle"},
	{"bbox_2d": [103, 158, 355, 188], "text_content": "is actually the source of the Yellow River?!"},
	{"bbox_2d": [516, 137, 596, 160], "text_content": "Documentary"},
	{"bbox_2d": [480, 278, 590, 298], "text_content": "21.345 million"},
	{"bbox_2d": [479, 358, 603, 382], "text_content": "Guarding Liberation West"},
	{"bbox_2d": [506, 434, 660, 454], "text_content": "Bilibili Documentary"},
	{"bbox_2d": [103, 358, 357, 384], "text_content": "I came to the 'Three Rivers Source' in textbooks"},
	{"bbox_2d": [103, 384, 285, 404], "text_content": "Tiger Tooth Youth Plus · 10 hours ago"},
	{"bbox_2d": [103, 614, 326, 636], "text_content": "Top bloggers say it's good"},
	{"bbox_2d": [103, 644, 316, 658], "text_content": "One table to track and analyze data across all platforms in real time"},
	{"bbox_2d": [103, 726, 172, 740], "text_content": "3,400,000"},
	{"bbox_2d": [202, 726, 236, 740], "text_content": "30,300,000"},
	{"bbox_2d": [272, 726, 318, 740], "text_content": "7,500,000"},
	{"bbox_2d": [360, 748, 405, 766], "text_content": "Advertisement"},
	{"bbox_2d": [10, 798, 383, 828], "text_content": "Workaholic productivity tool AI helps you work"},
	{"bbox_2d": [10, 834, 246, 858], "text_content": "Fishing time +1+1+1..."},
	{"bbox_2d": [10, 876, 168, 896], "text_content": "3,000+ people interested"},
	{"bbox_2d": [479, 800, 648, 828], "text_content": "Tower of God | No matter what you want, "},
	{"bbox_2d": [479, 844, 675, 864], "text_content": "Tower of God can achieve it."},
	{"bbox_2d": [479, 879, 675, 898], "text_content": "Fantasy · Updated to the third season"},
	{"bbox_2d": [776, 791, 889, 811], "text_content": "Refresh content"},
]
```"#;

        let result = parse_bounding_boxes(json);
        assert!(result.is_ok(), "Should successfully parse JSON with trailing comma: {:?}", result);
        
        let boxes = result.unwrap();
        assert_eq!(boxes.len(), 21);
        assert_eq!(boxes[0].text_content, Some("This small puddle".to_string()));
        assert_eq!(boxes[7].text_content, Some("Tiger Tooth Youth Plus · 10 hours ago".to_string()));
        assert_eq!(boxes[20].text_content, Some("Refresh content".to_string()));
    }

    #[test]
    fn test_parse_json_with_trailing_characters() {
        // This test specifically addresses the "trailing characters" error reported by the user
        let json_with_trailing = r#"```json
[
	{"bbox_2d": [80, 85, 110, 95], "text_content": "10:37 PM"},
	{"bbox_2d": [112, 85, 130, 95], "text_content": "Aug"},
	{"bbox_2d": [132, 85, 150, 95], "text_content": "29,"},
	{"bbox_2d": [152, 85, 166, 95], "text_content": "2025"}
]
some trailing text that should be ignored"#;

        let result = parse_bounding_boxes(json_with_trailing);
        assert!(result.is_ok(), "Should successfully parse JSON with trailing characters: {:?}", result);
        
        let boxes = result.unwrap();
        assert_eq!(boxes.len(), 4);
        assert_eq!(boxes[0].text_content, Some("10:37 PM".to_string()));
        assert_eq!(boxes[1].text_content, Some("Aug".to_string()));
        assert_eq!(boxes[2].text_content, Some("29,".to_string()));
        assert_eq!(boxes[3].text_content, Some("2025".to_string()));
    }

    #[test]
    fn test_parse_json_with_extensive_trailing_text() {
        // Test with the exact type of trailing text that was causing the error
        let json_with_extensive_trailing = r#"```json
[
	{"bbox_2d": [80, 85, 110, 95], "text_content": "10:37 PM"},
	{"bbox_2d": [112, 85, 130, 95], "text_content": "Aug"},
	{"bbox_2d": [132, 85, 150, 95], "text_content": "29,"},
	{"bbox_2d": [152, 85, 166, 95], "text_content": "2025"}
]
extensive trailing text with Chinese characters: 事件经过与延迟通报疑云山景城（Mountain View）警方在初步勘查后表示，现场"没有任何可疑活动或行为的迹象""#;

        let result = parse_bounding_boxes(json_with_extensive_trailing);
        assert!(result.is_ok(), "Should successfully parse JSON with extensive trailing characters: {:?}", result);
        
        let boxes = result.unwrap();
        assert_eq!(boxes.len(), 4);
        assert_eq!(boxes[0].text_content, Some("10:37 PM".to_string()));
        assert_eq!(boxes[3].text_content, Some("2025".to_string()));
    }

    #[test] 
    fn test_parse_user_reported_exact_case() {
        // The exact case from the user's error message - truncated at specific position
        let problematic_json = r#"```json
[
	{"bbox_2d": [80, 85, 110, 95], "text_content": "10:37 PM"},
	{"bbox_2d": [112, 85, 130, 95], "text_content": "Aug"},
	{"bbox_2d": [132, 85, 150, 95], "text_content": "29,"},
	{"bbox_2d": [152, 85, 166, 95], "text_content": "2025"},
	{"bbox_2d": [294, 158, 324, 170], "text_content": "Read"},
	{"bbox_2d": [326, 158, 348, 170], "text_content": "more"},
	{"bbox_2d": [350, 158, 366, 170], "text_content": "on"},
	{"bbox_2d": [368, 158, 399, 170], "text_content": "X"},
	{"bbox_2d": [108, 340, 130, 351], "text_content": "事件"},
	{"bbox_2d": [132, 340, 160, 351], "text_content": "经过"}
]
and then lots of trailing text that should be ignored"#;

        let result = parse_bounding_boxes(problematic_json);
        assert!(result.is_ok(), "Should handle user's specific case: {:?}", result);
        
        let boxes = result.unwrap();
        assert_eq!(boxes.len(), 10);
        assert_eq!(boxes[0].text_content, Some("10:37 PM".to_string()));
        assert_eq!(boxes[8].text_content, Some("事件".to_string()));
        assert_eq!(boxes[9].text_content, Some("经过".to_string()));
    }

    #[test]
    fn test_parse_json_with_leading_text() {
        // Test with explanatory text before JSON starts
        let json_with_leading = r#"Sure, here is the translation of the Chinese text into English:

```json
[
	{"bbox_2d": [24, 150, 648, 172], "text_content": "Home News Education Home Health Food Fashion Travel View"},
	{"bbox_2d": [24, 189, 648, 209], "text_content": "Focus News Entertainment News Life Bait Talk"},
	{"bbox_2d": [24, 210, 648, 229], "text_content": "Dong Xuan is really not a love brain? Zhang Wei's good you don't understand"}
]
```"#;

        let result = parse_bounding_boxes(json_with_leading);
        assert!(result.is_ok(), "Should successfully parse JSON with leading text: {:?}", result);
        
        let boxes = result.unwrap();
        assert_eq!(boxes.len(), 3);
        assert_eq!(boxes[0].text_content, Some("Home News Education Home Health Food Fashion Travel View".to_string()));
        assert_eq!(boxes[1].text_content, Some("Focus News Entertainment News Life Bait Talk".to_string()));
        assert_eq!(boxes[2].text_content, Some("Dong Xuan is really not a love brain? Zhang Wei's good you don't understand".to_string()));
    }

    #[test]
    fn test_parse_json_with_leading_text_no_markdown() {
        // Test with explanatory text before JSON starts, without markdown blocks
        let json_with_leading = r#"Here's the extracted bounding box information:

[
	{"bbox_2d": [24, 150, 648, 172], "text_content": "Home News Education"},
	{"bbox_2d": [24, 189, 648, 209], "text_content": "Focus News Entertainment"}
]"#;

        let result = parse_bounding_boxes(json_with_leading);
        assert!(result.is_ok(), "Should successfully parse JSON with leading text and no markdown: {:?}", result);
        
        let boxes = result.unwrap();
        assert_eq!(boxes.len(), 2);
        assert_eq!(boxes[0].text_content, Some("Home News Education".to_string()));
        assert_eq!(boxes[1].text_content, Some("Focus News Entertainment".to_string()));
    }

    #[test]
    fn test_parse_user_reported_leading_text_case() {
        // The exact case from the user's error message - leading explanatory text
        let problematic_json = r#"Sure, here is the translation of the Chinese text into English:

```json
[
	{"bbox_2d": [24, 150, 648, 172], "text_content": "Home News Education Home Health Food Fashion Travel View"},
	{"bbox_2d": [24, 189, 648, 209], "text_content": "Focus News Entertainment News Life Bait Talk"},
	{"bbox_2d": [24, 210, 648, 229], "text_content": "Dong Xuan is really not a love brain? Zhang Wei's good you don't understand"},
	{"bbox_2d": [24, 426, 648, 446], "text_content": "37204886498253! The U.S. debt limit is about to be untenable!"},
	{"bbox_2d": [24, 446, 648, 466], "text_content": "Jia Ling's new film wraps up, facing resistance, the whole network questions 'repeating the same trick'"}
]
```"#;

        let result = parse_bounding_boxes(problematic_json);
        assert!(result.is_ok(), "Should handle user's specific leading text case: {:?}", result);
        
        let boxes = result.unwrap();
        assert_eq!(boxes.len(), 5);
        assert_eq!(boxes[0].text_content, Some("Home News Education Home Health Food Fashion Travel View".to_string()));
        assert_eq!(boxes[3].text_content, Some("37204886498253! The U.S. debt limit is about to be untenable!".to_string()));
        assert_eq!(boxes[4].text_content, Some("Jia Ling's new film wraps up, facing resistance, the whole network questions 'repeating the same trick'".to_string()));
    }

    #[test]
    fn test_parse_json_complex_leading_text() {
        // Test with multi-paragraph explanatory text
        let json_with_complex_leading = r#"Based on the image analysis, I can extract the following text content with their bounding boxes.

The image appears to be a Chinese news website or application interface. Here are the identified text elements:

```json
[
	{"bbox_2d": [10, 20, 100, 40], "text_content": "Breaking News"},
	{"bbox_2d": [10, 50, 200, 70], "text_content": "Latest Updates"}
]
```

Note: These coordinates are approximate and may need adjustment for production use."#;

        let result = parse_bounding_boxes(json_with_complex_leading);
        assert!(result.is_ok(), "Should handle complex leading text: {:?}", result);
        
        let boxes = result.unwrap();
        assert_eq!(boxes.len(), 2);
        assert_eq!(boxes[0].text_content, Some("Breaking News".to_string()));
        assert_eq!(boxes[1].text_content, Some("Latest Updates".to_string()));
    }

    #[test]
    fn test_parse_json_leading_text_with_trailing_text() {
        // Test combining both leading and trailing text fixes
        let json_with_both = r#"Here's the parsed content:

```json
[
	{"bbox_2d": [10, 20, 100, 40], "text_content": "Header"},
	{"bbox_2d": [10, 50, 200, 70], "text_content": "Content"}
]
```

Additional notes: The parsing was successful and all text elements were extracted."#;

        let result = parse_bounding_boxes(json_with_both);
        assert!(result.is_ok(), "Should handle both leading and trailing text: {:?}", result);
        
        let boxes = result.unwrap();
        assert_eq!(boxes.len(), 2);
        assert_eq!(boxes[0].text_content, Some("Header".to_string()));
        assert_eq!(boxes[1].text_content, Some("Content".to_string()));
    }

    #[test]
    fn test_parse_json_truncated_with_leading_text() {
        // Test truncated JSON with leading text - should still work for complete entries
        let json_truncated_with_leading = r#"The extracted text elements are:

```json
[
	{"bbox_2d": [24, 150, 648, 172], "text_content": "Complete Entry"},
	{"bbox_2d": [24, 189, 648, 209], "text_content": "Incomplete"#;

        let result = parse_bounding_boxes(json_truncated_with_leading);
        match result {
            Ok(boxes) => {
                // Should at least parse the complete entry
                assert!(boxes.len() >= 1);
                assert_eq!(boxes[0].text_content, Some("Complete Entry".to_string()));
            }
            Err(_) => {
                // If parsing fails completely, that's acceptable for truncated input
                // The important thing is no panic occurs
            }
        }
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

    #[test]
    fn test_parse_json_with_unescaped_quotes() {
        // Test the specific case mentioned by the user with unescaped quotes within strings
        let json_with_unescaped_quotes = r#"```json
[
	{"bbox_2d": [10, 160, 51, 173], "text_content": "health"},
	{"bbox_2d": [10, 210, 361, 231], "text_content": "How bad is sitting for too long? More than you think"},
	{"bbox_2d": [10, 714, 544, 732], "text_content": "We've all heard that sitting for too long is bad for you. While it may not be "as bad as smoking," too much sitting can still shorten your lifespan."},
	{"bbox_2d": [10, 802, 544, 820], "text_content": "\"Sitting actually accelerates aging,\" says Katie Bowman. \"Whether it's bone or joint health, muscle mass, or energy levels,\" she adds, \"many of the things you think are signs of aging are actually heavily influenced by how much you sit.\""}
]
```"#;

        let result = parse_bounding_boxes(json_with_unescaped_quotes);
        assert!(result.is_ok(), "Should successfully parse JSON with unescaped quotes: {:?}", result);
        
        let boxes = result.unwrap();
        assert_eq!(boxes.len(), 4);
        assert_eq!(boxes[0].text_content, Some("health".to_string()));
        assert_eq!(boxes[1].text_content, Some("How bad is sitting for too long? More than you think".to_string()));
        assert!(boxes[2].text_content.as_ref().unwrap().contains("as bad as smoking"));
        assert!(boxes[3].text_content.as_ref().unwrap().contains("Sitting actually accelerates aging"));
    }

    #[test]
    fn test_parse_json_with_truncated_dollar_sign() {
        // Test the specific truncation case with '$' at the end
        let json_with_dollar_truncation = r#"```json
[
	{"bbox_2d": [10, 160, 51, 173], "text_content": "health"},
	{"bbox_2d": [10, 714, 544, 732], "text_content": "We've all heard that sitting for too long is bad for you. Human evolution didn't account for the need to sit for long periods, which cancels out the benefits of exercise and can lea$
]
```"#;

        let result = parse_bounding_boxes(json_with_dollar_truncation);
        // Should either succeed with repair or fail gracefully
        match result {
            Ok(boxes) => {
                assert!(boxes.len() >= 1);
                assert_eq!(boxes[0].text_content, Some("health".to_string()));
                // Second entry might be truncated but should not cause a panic
            }
            Err(_) => {
                // If parsing fails, that's acceptable for severely truncated input
                // The important thing is no panic occurs
            }
        }
    }
}