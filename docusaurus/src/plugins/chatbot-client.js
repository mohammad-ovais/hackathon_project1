/**
 * Client-side script for the RAG Chatbot plugin
 * This script loads the chatbot widget and makes it available globally
 */

// Load the chatbot widget script dynamically
function loadChatWidget(backendUrl) {
  return new Promise((resolve, reject) => {
    // Check if widget is already loaded
    if (window.ChatWidget) {
      resolve(window.ChatWidget);
      return;
    }

    // Create script element
    const script = document.createElement('script');
    script.src = '/js/chat-widget.js'; // Updated path to where the widget will be hosted

    // Add data attributes for configuration
    if (backendUrl) {
      script.setAttribute('data-backend-url', backendUrl);
      script.setAttribute('data-theme', 'light');
      script.setAttribute('data-position', 'right');
    }

    script.async = true;
    script.onload = () => {
      if (window.ChatWidget) {
        resolve(window.ChatWidget);
      } else {
        reject(new Error('ChatWidget not available after loading script'));
      }
    };
    script.onerror = () => {
      reject(new Error('Failed to load chat widget script'));
    };

    document.head.appendChild(script);
  });
}

// Initialize the chatbot when the page is ready
if (typeof window !== 'undefined') {
  // Wait for DOM to be ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', async () => {
      try {
        // Attempt to load the chat widget
        await loadChatWidget(window.chatbotConfig?.backendUrl);
        console.log('Chatbot widget loaded successfully');
      } catch (error) {
        console.error('Failed to load chatbot widget:', error);
      }
    });
  } else {
    // DOM already ready, load immediately
    setTimeout(async () => {
      try {
        await loadChatWidget(window.chatbotConfig?.backendUrl);
        console.log('Chatbot widget loaded successfully');
      } catch (error) {
        console.error('Failed to load chatbot widget:', error);
      }
    }, 0);
  }
}

// Export for use in other modules if needed
export { loadChatWidget };