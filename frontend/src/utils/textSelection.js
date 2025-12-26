/**
 * Utility functions for text selection handling
 */

// Get selected text from the page
function getSelectedText() {
  return window.getSelection ? window.getSelection().toString().trim() : '';
}

// Get the context around selected text
function getSelectionContext() {
  const selection = window.getSelection();
  if (!selection.rangeCount) return null;

  const range = selection.getRangeAt(0);
  const preContext = range.startContainer.textContent.substring(0, range.startOffset);
  const postContext = range.startContainer.textContent.substring(range.endOffset);

  // Get up to 100 characters before and after the selection
  const pre = preContext.length > 100 ? preContext.slice(-100) : preContext;
  const post = postContext.length > 100 ? postContext.slice(0, 100) : postContext;

  return {
    pre,
    selected: selection.toString(),
    post
  };
}

// Add event listener for text selection
function addSelectionListener(callback) {
  document.addEventListener('mouseup', () => {
    const selectedText = getSelectedText();
    if (selectedText) {
      callback(selectedText);
    }
  });
}

// Check if text is selected
function isTextSelected() {
  return getSelectedText().length > 0;
}

export {
  getSelectedText,
  getSelectionContext,
  addSelectionListener,
  isTextSelected
};