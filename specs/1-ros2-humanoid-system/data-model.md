# Data Model: ROS 2 for Humanoid Robotics Education

## Content Structure

### Chapter Entity
- **name**: String (required) - The name of the chapter
- **slug**: String (required) - URL-friendly identifier
- **title**: String (required) - Display title for the chapter
- **description**: String (optional) - Brief description of the chapter content
- **prerequisites**: Array of String (optional) - Knowledge required before reading
- **learning_objectives**: Array of String (required) - What the reader should learn
- **content**: String (required) - The main content in Markdown format
- **related_topics**: Array of String (optional) - Related topics or chapters
- **examples**: Array of Object (optional) - Code or practical examples
  - example_type: String (code, diagram, simulation)
  - content: String (the actual example content)
  - explanation: String (explanation of the example)

### Example Entity (for code examples)
- **title**: String (required) - Brief title of the example
- **description**: String (optional) - Explanation of what the example demonstrates
- **code**: String (required) - The actual code example
- **language**: String (required) - Programming language (e.g., Python, XML for URDF)
- **output**: String (optional) - Expected output or behavior
- **explanation**: String (required) - Explanation of the code

### Section Entity (for organizing content within chapters)
- **title**: String (required) - Title of the section
- **slug**: String (required) - URL-friendly identifier for the section
- **content**: String (required) - Content of the section in Markdown
- **order**: Integer (required) - Order of the section within the chapter
- **prerequisites**: Array of String (optional) - What needs to be understood first

## Navigation Structure

### Sidebar Item Entity
- **type**: String (required) - Type of sidebar item (doc, category, link)
- **label**: String (required) - Display label in sidebar
- **id**: String (optional) - Reference to a document
- **items**: Array of Object (optional) - Child items if category type
- **collapsed**: Boolean (optional) - Whether the category is collapsed by default

## Content Relationships

1. **Module** (ROS 2 for Humanoid Robotics) contains multiple **Chapters**
2. Each **Chapter** contains multiple **Sections**
3. Each **Chapter** may contain multiple **Examples**
4. **Chapters** may have relationships with other **Chapters** as related topics

## Validation Rules

- Each chapter must have a unique slug within the module
- Each chapter must have at least one learning objective
- Each chapter must have content (not empty)
- All referenced related topics must exist in the documentation
- Code examples must specify a valid programming language
- Section order values must be unique within each chapter