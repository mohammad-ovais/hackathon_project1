# Integrated RAG Chatbot for Physical AI & Humanoid Robotics Textbook

## Feature Overview
Create an integrated RAG (Retrieval-Augmented Generation) chatbot that allows users to ask questions about the Physical AI & Humanoid Robotics textbook content. The chatbot should provide accurate answers with citations to specific parts of the textbook.

## Functional Requirements
1. **Textbook Content Integration**: The chatbot should have access to the full Physical AI & Humanoid Robotics textbook content
2. **Question Answering**: Users can ask questions about the textbook content and receive accurate answers
3. **Selected Text Queries**: Users can select text in the textbook and ask questions about the selected portion
4. **Citation Generation**: Answers should include citations to specific sections/pages of the textbook
5. **Real-time Interaction**: The chatbot should respond to queries in real-time (under 3 seconds)
6. **Embeddable Widget**: The chatbot should be embeddable on all book pages via a JavaScript widget

## Non-Functional Requirements
1. **Free Tier Compliance**: System must operate within free tier limits of all services (Gemini, Qdrant, hosting)
2. **No User Data Storage**: No persistent storage of user queries or personal data
3. **Privacy**: User queries should not be stored or logged
4. **Performance**: Responses should be delivered in under 3 seconds
5. **Accuracy**: Answers must be grounded in the textbook content
6. **Availability**: System should be available 99% of the time during active use

## Technical Constraints
1. Use only free-tier services (Gemini API, Qdrant Cloud, GitHub Pages)
2. No backend persistent storage of user data
3. Docusaurus-based documentation site
4. JavaScript/TypeScript frontend widget
5. FastAPI backend for orchestration
6. Qdrant Cloud for vector storage

## Acceptance Criteria
1. Users can ask questions about the textbook and receive accurate answers
2. Selected text queries work correctly
3. Citations are provided with each answer
4. Response time is under 3 seconds
5. System operates within free tier limits
6. No user data is persisted
7. Widget is embedded on all book pages