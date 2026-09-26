#include "solvers/openfoam/FoamFieldReader.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace solvers::foam {

namespace {

class Cursor {
public:
    Cursor(const std::string& text, std::string file) : p_(text.c_str()), end_(text.c_str() + text.size()), file_(std::move(file)) {}

    void skipSpace()
    {
        for (;;) {
            while (p_ < end_ && std::isspace(static_cast<unsigned char>(*p_)))
                ++p_;
            if (p_ + 1 < end_ && p_[0] == '/' && p_[1] == '/') {
                while (p_ < end_ && *p_ != '\n')
                    ++p_;
            } else if (p_ + 1 < end_ && p_[0] == '/' && p_[1] == '*') {
                p_ += 2;
                while (p_ + 1 < end_ && !(p_[0] == '*' && p_[1] == '/'))
                    ++p_;
                p_ = std::min(p_ + 2, end_);
            } else {
                return;
            }
        }
    }

    std::string word()
    {
        skipSpace();
        const char* s = p_;
        while (p_ < end_ && !std::isspace(static_cast<unsigned char>(*p_)) && *p_ != '(' && *p_ != '{' && *p_ != ';')
            ++p_;
        return {s, p_};
    }

    char peek()
    {
        skipSpace();
        return p_ < end_ ? *p_ : '\0';
    }

    void expect(char c)
    {
        if (peek() != c)
            fail(std::string("expected '") + c + "'");
        ++p_;
    }

    double number()
    {
        skipSpace();
        char* e = nullptr;
        const double v = std::strtod(p_, &e);
        if (e == p_)
            fail("expected a number");
        p_ = e;
        return v;
    }

    long integer()
    {
        skipSpace();
        char* e = nullptr;
        const long v = std::strtol(p_, &e, 10);
        if (e == p_)
            fail("expected a list size");
        p_ = e;
        return v;
    }

    glm::vec3 vec()
    {
        expect('(');
        glm::vec3 v;
        v.x = static_cast<float>(number());
        v.y = static_cast<float>(number());
        v.z = static_cast<float>(number());
        expect(')');
        return v;
    }

    // Position just after the first top-level occurrence of `key` as a word.
    void seekKeyword(const std::string& key)
    {
        const std::string text(p_, end_);
        std::size_t pos = 0;
        while ((pos = text.find(key, pos)) != std::string::npos) {
            const bool startOk = pos == 0 || std::isspace(static_cast<unsigned char>(text[pos - 1]));
            const std::size_t after = pos + key.size();
            const bool endOk = after < text.size() && std::isspace(static_cast<unsigned char>(text[after]));
            if (startOk && endOk) {
                p_ += after;
                return;
            }
            pos = after;
        }
        fail("no '" + key + "' entry");
    }

    [[noreturn]] void fail(const std::string& what) const
    {
        throw std::runtime_error("OpenFOAM field " + file_ + ": " + what);
    }

private:
    const char* p_;
    const char* end_;
    std::string file_;
};

std::string readFile(const std::filesystem::path& file)
{
    std::ifstream in(file, std::ios::binary);
    if (!in)
        throw std::runtime_error("OpenFOAM: cannot open field file " + file.string());
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

void checkAscii(const std::string& text, const std::filesystem::path& file)
{
    const auto header = text.find("FoamFile");
    if (header == std::string::npos)
        return;
    const auto close = text.find('}', header);
    const auto fmt = text.find("format", header);
    if (fmt != std::string::npos && fmt < close) {
        const auto semi = text.find(';', fmt);
        if (text.substr(fmt, semi - fmt).find("binary") != std::string::npos)
            throw std::runtime_error("OpenFOAM field " + file.string() + " is binary; expected writeFormat ascii");
    }
}

template <typename T, typename ReadOne>
std::vector<T> readField(const std::filesystem::path& file, std::size_t expected, ReadOne readOne)
{
    const std::string text = readFile(file);
    checkAscii(text, file);
    Cursor c(text, file.string());
    c.seekKeyword("internalField");
    const std::string kind = c.word();
    std::vector<T> out;
    if (kind == "uniform") {
        out.assign(expected > 0 ? expected : 1, readOne(c));
        return out;
    }
    if (kind != "nonuniform")
        c.fail("unknown internalField form '" + kind + "'");
    const std::string type = c.word(); // List<vector> / List<scalar>
    if (type.rfind("List<", 0) != 0)
        c.fail("unexpected list type '" + type + "'");
    const long n = c.integer();
    if (n < 0 || (expected > 0 && static_cast<std::size_t>(n) != expected))
        c.fail("list has " + std::to_string(n) + " values, expected " + std::to_string(expected));
    if (c.peek() == '{') {
        c.expect('{');
        out.assign(static_cast<std::size_t>(n), readOne(c));
        c.expect('}');
        return out;
    }
    c.expect('(');
    out.reserve(static_cast<std::size_t>(n));
    for (long i = 0; i < n; ++i)
        out.push_back(readOne(c));
    c.expect(')');
    return out;
}

} // namespace

std::vector<glm::vec3> readVectorField(const std::filesystem::path& file, std::size_t expectedCount)
{
    return readField<glm::vec3>(file, expectedCount, [](Cursor& c) { return c.vec(); });
}

std::vector<float> readScalarField(const std::filesystem::path& file, std::size_t expectedCount)
{
    return readField<float>(file, expectedCount, [](Cursor& c) { return static_cast<float>(c.number()); });
}

} // namespace solvers::foam
