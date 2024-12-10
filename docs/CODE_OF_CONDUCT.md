# C++ Code of Conduct

This document outlines the coding standards and best practices for writing C++ code in our project, inspired by Google's C++ Style Guide with some custom convention.

## 1. General Principles

- **Consistency**: Code should be consistent with the style and conventions defined in this document.
- **Readability**: Write code that is easy to read and understand. Use meaningful names and maintain a clear structure.
- **Simplicity**: Prefer simple and straightforward solutions. Avoid unnecessary complexity.
- **Efficiency**: Ensure the code is efficient in terms of both time and space.
- **Portability**: Write portable code that can run on multiple platforms without modification.

## 2. Naming Conventions

### 2.1 Variables

- **Global Variables**: `g_global_variable`
- **Member Variables**: `m_member_variable`
- **Local Variables**: `some_local`
- **Constants**: `kConstantName`

### 2.2 Functions

- Function names should be written in mixed case: `functionName`
- Member functions should follow the same convention: `functionName`

### 2.3 Classes and Structs

- Class names should be written in mixed case: `ClassName`
- Struct names should also follow the mixed case convention: `StructName`

### 2.4 Macros

- Macros should be written in all uppercase with underscores: `MY_MACRO`

## 3. Formatting

### 3.1 Indentation

- Use 2 spaces for indentation.
- Do not use tabs.

### 3.2 Line Length

- Limit lines to 120 characters.

### 3.3 Braces

- Braces should be used for all control structures and placed on the same line as the control statement.

```cpp
if (condition) {
  some_local = value;
} else {
  some_local = other_value;
}
```

### 3.4 Spacing

Use a single space after keywords and around operators.

```cpp
for (int i = 0; i < n; ++i) {
  some_local += i;
}
```

## 4. Comments

### 4.1 File Comments

4.1 File Comments
Each file should start with a license header and a brief description of its contents.

```cpp
Copyright (c) 202X, WiseVision. All rights reserved.
Some brief description of the file.
```


### 4.2 Function Comments

Functions should have comments describing their purpose, parameters, and return values.

```cpp
// Computes the factorial of a number.
//
// @param n The number to compute the factorial of.
//
// @return The factorial of n.

int factorial(int n) {
  // Implementation
}
```

### 4.3 Inline Comments

- Use inline comments sparingly and only for complex or non-obvious code.

```cpp
int sum = 0;  // Initialize sum.
for (int i = 0; i < n; ++i) {
  sum += i;  // Add each number to sum.
}
```

### 4.4 TODO Comments

- Use `TODO` comments to indicate areas of the code that need to be improved or completed. Add a GitHub username to indicate who should address the issue.

```cpp
void someFunction() {
    return;  // TODO(adkrawcz): Implement this function.
}
```

## 5. Best Practices

### 5.1 Avoid Magic Numbers

Avoid using magic numbers in the code. Instead, use named constants.

```cpp
const int kMaxIterations = 1000;
```

### 5.2 Use of auto

- Prefer explicit types over auto, except when the type is obvious from the context or where it significantly improves code clarity.

```cpp
int some_local = 42;  // Preferred
auto some_local = getAnswer();  // Acceptable if getAnswer's return type is clear
```

### 5.3 Smart Pointers

- Prefer using smart pointers (e.g., `std::unique_ptr`, `std::shared_ptr`) over raw pointers.

### 5.4 Error Handling

- Always check for errors and handle them appropriately. Avoid using exceptions for control flow, avoid using return codes for error handling.

```cpp
try {
  some_local = performOperation();
} catch (const std::exception& e) {
  std::cerr << "Error: " << e.what() << std::endl;
}
```

- Use early returns to handle errors and reduce nesting.

```cpp
if (!IsValid(input)) {
  return false;
}
```

### 5.5 Const Correctness

- Use `const` wherever possible to indicate that a variable is not modified.

```cpp
int getAnswer() const {
  return some_local;
}
```

### 5.6 Avoid Global Variables

- Avoid using global variables. Prefer passing variables as arguments to functions.