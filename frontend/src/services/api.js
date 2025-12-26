/**
 * API service for communicating with the backend
 */

class ApiService {
  constructor(backendUrl) {
    this.backendUrl = backendUrl;
  }

  async queryChat(query, sessionId, includeCitations = true) {
    try {
      const response = await fetch(`${this.backendUrl}/api/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query,
          session_id: sessionId,
          include_citations: includeCitations
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error in queryChat:', error);
      throw error;
    }
  }

  async querySelection(query, selectedText, sessionId, includeCitations = true) {
    try {
      const response = await fetch(`${this.backendUrl}/api/chat/query-selection`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query,
          selected_text: selectedText,
          session_id: sessionId,
          include_citations: includeCitations
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error in querySelection:', error);
      throw error;
    }
  }

  async healthCheck() {
    try {
      const response = await fetch(`${this.backendUrl}/api/health`);

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error in healthCheck:', error);
      throw error;
    }
  }
}

export default ApiService;