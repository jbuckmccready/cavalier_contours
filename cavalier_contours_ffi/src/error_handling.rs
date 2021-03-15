use std::{cell::RefCell, ffi::CString};

/// Holds last error information data.
pub struct LastErrorData {
    /// Human readable error message.
    pub error_msg: CString,
    /// Data associated with the last error that occurred, may have information about the function
    /// called and inputs received.
    pub error_report_data: CString,
}

impl LastErrorData {
    pub fn new(error_msg: CString, error_report_data: CString) -> Self {
        LastErrorData {
            error_msg,
            error_report_data,
        }
    }
}

// Storage for last error set
thread_local!(pub static LAST_ERROR: RefCell<Option<LastErrorData>> = RefCell::new(None));

/// Set last error information.
///
/// `error_msg` and `error_report` are turned into CStrings from bytes, bytes must not include any
/// nulls.
pub fn set_last_error<T: Into<Vec<u8>>>(error_msg: T, error_report: T) {
    let msg = CString::new(error_msg)
        .unwrap_or_else(|_| CString::new("Failed to create error message string!").unwrap());
    let report = CString::new(error_report)
        .unwrap_or_else(|_| CString::new("Failed to create error report string!").unwrap());

    LAST_ERROR.with(|last_result| {
        *last_result.borrow_mut() = Some(LastErrorData::new(msg, report));
    });
}
