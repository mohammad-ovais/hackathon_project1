/**
 * Main entry point for the chat widget bundle
 */
import ChatWidget from './components/ChatWidget';

// Create a global API for the widget
const ChatWidgetAPI = {
  init: (config) => {
    // Initialize the widget with provided configuration
    // This could involve setting up event listeners, etc.
    console.log('ChatWidget initialized with config:', config);

    // Store config globally for use by the widget
    window.chatbotConfig = config;
  },

  open: () => {
    // Method to programmatically open the widget
    const event = new CustomEvent('chatWidgetOpen');
    document.dispatchEvent(event);
  },

  close: () => {
    // Method to programmatically close the widget
    const event = new CustomEvent('chatWidgetClose');
    document.dispatchEvent(event);
  }
};

// Make the API available globally
window.ChatWidget = ChatWidgetAPI;

export default ChatWidgetAPI;